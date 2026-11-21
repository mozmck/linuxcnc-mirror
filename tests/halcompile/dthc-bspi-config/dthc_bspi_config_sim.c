/* Host-side simulation of ccc_dthc_bspi THC-config handling (test.sh
 * builds it with the module flags and COMP_C naming the generated C).
 * The driver's generated C is included; HAL, RTAPI and hm2 calls are
 * stubbed.  A fake DTHC6 answers each SPI frame one transfer later, so a
 * reply is parsed two servo cycles after its request.  It acknowledges each
 * CONF_SET and saves as firmware 3.1 does: save_quiet cycles after the last
 * write, dropping the link for save_gap cycles (the flash erases, with
 * interrupts masked).  A test makes a frame land in a save with busy (the
 * next frames are lost) or gap_next (the save starts with the next CONF_SET,
 * so its ack is lost).  "./sim motor" runs the motor-settings checks. */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rtapi.h"
#include "hal.h"
#include <hostmot2-serial.h>

/* ---- pin registry ---- */
typedef struct { char name[128]; void *ptr; } pinrec_t;
static pinrec_t pins[512];
static int npins;

static int newpin(void **addr, const char *fmt, va_list ap)
{
    pinrec_t *p = &pins[npins++];
    vsnprintf(p->name, sizeof p->name, fmt, ap);
    void *mem = calloc(1, 16);
    *addr = mem;
    p->ptr = mem;
    return 0;
}
#define PIN_STUB(fn, T) \
int fn(hal_pin_dir_t dir, T **data_ptr_addr, int comp_id, const char *fmt, ...) \
{ va_list ap; va_start(ap, fmt); int r = newpin((void **)data_ptr_addr, fmt, ap); va_end(ap); (void)dir; (void)comp_id; return r; }
PIN_STUB(hal_pin_u32_newf, hal_u32_t)
PIN_STUB(hal_pin_s32_newf, hal_s32_t)
PIN_STUB(hal_pin_float_newf, hal_float_t)
PIN_STUB(hal_pin_bit_newf, hal_bit_t)

static void *pin(const char *suffix)
{
    size_t ls = strlen(suffix);
    for (int i = 0; i < npins; i++) {
        size_t ln = strlen(pins[i].name);
        if (ln >= ls && !strcmp(pins[i].name + ln - ls, suffix) &&
            (ln == ls || pins[i].name[ln - ls - 1] == '.'))
            return pins[i].ptr;
    }
    fprintf(stderr, "no pin %s\n", suffix);
    exit(2);
}

void *hal_malloc(long int size) { return calloc(1, size); }
int hal_init(const char *name) { (void)name; return 1; }
int hal_ready(int comp_id) { (void)comp_id; return 0; }
int hal_exit(int comp_id) { (void)comp_id; return 0; }
int hal_stream_create(hal_stream_t *s, int c, int k, int d, const char *t) { (void)s; (void)c; (void)k; (void)d; (void)t; return -1; }
void hal_stream_destroy(hal_stream_t *s) { (void)s; }
int hal_stream_write(hal_stream_t *s, union hal_stream_data *buf) { (void)s; (void)buf; return 0; }
void rtapi_print_msg(msg_level_t level, const char *fmt, ...)
{ va_list ap; va_start(ap, fmt); if (level <= RTAPI_MSG_ERR) vfprintf(stderr, fmt, ap); va_end(ap); }
static char shmem[65536];
int rtapi_shmem_new(int key, int module_id, unsigned long int size) { (void)key; (void)module_id; (void)size; return 1; }
int rtapi_shmem_getptr(int id, void **ptr) { (void)id; *ptr = shmem; return 0; }
int rtapi_shmem_delete(int id, int module_id) { (void)id; (void)module_id; return 0; }

static rtapi_u32 txw[12], rxw[12];
static int tram_i;
static int (*drv_read)(void *), (*drv_write)(void *);
static void *drv_sub;
int hm2_bspi_setup_chan(char *n, int c, int cs, int b, double m, int d, int cp, int ph, int cl, int e, int s)
{ (void)n; (void)c; (void)cs; (void)b; (void)m; (void)d; (void)cp; (void)ph; (void)cl; (void)e; (void)s; return 0; }
int hm2_bspi_set_read_function(char *n, int (*f)(void *), void *s) { (void)n; drv_read = f; drv_sub = s; return 0; }
int hm2_bspi_set_write_function(char *n, int (*f)(void *), void *s) { (void)n; drv_write = f; drv_sub = s; return 0; }
int hm2_allocate_bspi_tram(char *n) { (void)n; return 0; }
int hm2_tram_add_bspi_frame(char *n, int c, rtapi_u32 **w, rtapi_u32 **r)
{ (void)n; (void)c; *w = &txw[tram_i]; *r = &rxw[tram_i]; tram_i++; return 0; }
int hm2_bspi_clear_fifo(char *n) { (void)n; return 0; }

#include COMP_C

/* ---- fake firmware ---- */
static struct {
    uint16_t sp, base, delay_ms, upper, lower;
    uint8_t enable_autoset;
    int thc_writes;
    uint16_t last_written_sp;
    uint32_t steps, up, down, accel;
    uint8_t dir, step, unit;
    int motor_writes;
    int save_gap;   /* cycles each settings save keeps the link down */
    int save_quiet; /* cycles after the last write before it saves */
    int quiet;      /* cycles since the last write; -1 with no save due */
    int busy;       /* cycles left in the current save */
    int gap_next;   /* a save of this length starts with the next CONF_SET */
    uint8_t prof[DTHC_PROFILE_BYTES];    /* thc_profile_serialize() */
    uint8_t staging[DTHC_PROFILE_BYTES];
    uint8_t page_mask;
    int profile_commits;
    uint8_t reply[48];
} fw;

static void fw_frame(uint8_t *f, uint8_t cmd, const uint8_t *pl, uint8_t n)
{
    memset(f, 0, 48);
    f[0] = 0x55; f[1] = (uint8_t)(3 + n); f[2] = cmd;
    memcpy(f + 3, pl, n);
    f[3 + n] = dthc_crc8(f, 3 + n);
}
static void put16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v & 0xff; }
static void put32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }
static uint32_t get32(const uint8_t *p)
{ return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }

static void fw_process(const uint8_t *f)
{
    uint8_t pl[48] = {0};
    if (f[0] != 0x55) { fw_frame(fw.reply, 0, pl, 0); return; }
    uint8_t cmd = f[2];
    uint8_t n = (uint8_t)(f[1] - 3);
    const uint8_t *d = f + 3;
    switch (cmd) {
    case DTHC_SPI_CMD_CONFIG:
        pl[0] = 16;
        put16(pl + 1, fw.delay_ms);
        put16(pl + 3, fw.sp);
        put16(pl + 5, fw.upper);
        put16(pl + 7, fw.lower);
        put32(pl + 19, fw.steps);
        put32(pl + 23, fw.up);
        put32(pl + 27, fw.accel);
        pl[31] = fw.dir;
        pl[32] = fw.step;
        pl[33] = fw.unit;
        put32(pl + 34, fw.down);
        put32(pl + 38, DTHC_FW_REV);
        fw_frame(fw.reply, cmd, pl, 43);
        break;
    case DTHC_SPI_CMD_CONF_SET:
        if (d[0] == 1) {
            /* spi_handle_config_set() mode 1, then axis_calculate() */
            fw.motor_writes++;
            fw.steps = get32(d + 1);
            fw.up = get32(d + 5);
            fw.accel = get32(d + 9);
            fw.dir = d[13] ? 1 : 0;
            fw.step = d[14] ? 1 : 0;
            fw.unit = d[15];
            fw.down = (n > 16) ? get32(d + 16) : fw.up;
            if (fw.steps > 50000 || fw.steps < 10) fw.steps = 10000;
            if (fw.up > 50000 || fw.up < 5) fw.up = 300;
            if (fw.down > fw.up || fw.down < 5) fw.down = fw.up;
            if (fw.accel > 50000 || fw.accel < 2) fw.accel = 35;
        } else if (d[0] == 0) {
            uint16_t sp = get16(d + 4);   /* d[0] is the mode */
            fw.thc_writes++;
            fw.last_written_sp = sp;
            fw.delay_ms = get16(d + 2);
            fw.upper = get16(d + 6);
            fw.lower = get16(d + 8);
            if (sp != fw.sp || !fw.enable_autoset) {
                fw.sp = sp;
                if (fw.sp <= fw.lower || fw.sp >= fw.upper) fw.sp = 1920;
                fw.base = fw.sp;
            }
        }
        /* the ack: mode, the frame's CRC, applied */
        pl[0] = d[0]; pl[1] = f[f[1]]; pl[2] = 1;
        fw_frame(fw.reply, cmd, pl, 3);
        if (fw.gap_next) {
            fw.busy = fw.gap_next;
            fw.gap_next = 0;
        } else if (fw.save_gap) {
            fw.quiet = 0;
        }
        break;
    case DTHC_SPI_CMD_PROFILE: {
        /* spi_handle_profile(): 0 read page, 1 stage page, 2 commit, 3 discard */
        uint8_t op = n ? d[0] : 0xffu, page = (n > 1) ? d[1] : 0u;
        uint8_t status = 0, count = 0;
        uint16_t off = (uint16_t)page * DTHC_PROFILE_PAGE_BYTES;
        uint16_t expected = (DTHC_PROFILE_BYTES - off > DTHC_PROFILE_PAGE_BYTES) ?
            DTHC_PROFILE_PAGE_BYTES : (uint16_t)(DTHC_PROFILE_BYTES - off);
        if (op == 0) {
            if (page >= DTHC_PROFILE_PAGE_COUNT) status = 1;
            else {
                if (page == 0) memcpy(fw.staging, fw.prof, DTHC_PROFILE_BYTES);
                count = (uint8_t)expected;
            }
        } else if (op == 1) {
            if (page >= DTHC_PROFILE_PAGE_COUNT || n < 3 || d[2] != expected) status = 1;
            else {
                memcpy(fw.staging + off, d + 3, expected);
                fw.page_mask |= (uint8_t)(1u << page);
            }
        } else if (op == 2) {
            if (n < 5 || fw.page_mask != (1u << DTHC_PROFILE_PAGE_COUNT) - 1u) status = 1;
            else if (get32(d + 1) != dthc_crc32(fw.staging, DTHC_PROFILE_BYTES)) status = 3;
            else {
                memcpy(fw.prof, fw.staging, DTHC_PROFILE_BYTES);
                fw.page_mask = 0;
                fw.profile_commits++;
            }
        } else if (op == 3) {
            fw.page_mask = 0;
        } else {
            status = 1;
        }
        pl[0] = status; pl[1] = op; pl[2] = page;
        put16(pl + 3, DTHC_PROFILE_BYTES);
        put32(pl + 5, dthc_crc32(fw.staging, DTHC_PROFILE_BYTES));
        pl[9] = count;
        if (op == 0 && status == 0) memcpy(pl + 10, fw.staging + off, count);
        fw_frame(fw.reply, cmd, pl, (uint8_t)(10 + count));
        break;
    }
    case DTHC_SPI_CMD_DATA:
        if (d[0] & 0x10) fw.enable_autoset = 1;
        else { if (fw.enable_autoset) fw.base = fw.sp; fw.enable_autoset = 0; }
        put16(pl + 12, fw.base);
        fw_frame(fw.reply, cmd, pl, 14);
        break;
    default:
        fw_frame(fw.reply, 0, pl, 0);
        break;
    }
}

static uint8_t pending_reply[48];
static void cycle(void)
{
    /* servo read: what the last transfer clocked in */
    for (int i = 0; i < 12; i++)
        rxw[i] = (rtapi_u32)pending_reply[i*4] << 24 | (rtapi_u32)pending_reply[i*4+1] << 16 |
                 (rtapi_u32)pending_reply[i*4+2] << 8 | pending_reply[i*4+3];
    drv_read(drv_sub);
    drv_write(drv_sub);
    if (fw.quiet >= 0 && ++fw.quiet >= fw.save_quiet) {
        fw.busy = fw.save_gap;
        fw.quiet = -1;
    }
    /* A settings save: the frame is lost and the host reads no frame.
     * spi_comm_restart() then starts again from an idle frame. */
    if (fw.busy) {
        memset(pending_reply, 0, 48);
        if (--fw.busy == 0) {
            uint8_t none[1] = {0};
            fw_frame(fw.reply, 0, none, 0);
        }
        return;
    }
    /* transfer: the firmware clocks out its prepared reply while it
     * receives this frame, then prepares the reply to it */
    memcpy(pending_reply, fw.reply, 48);
    uint8_t f[48];
    for (int i = 0; i < 12; i++) {
        f[i*4] = txw[i] >> 24; f[i*4+1] = txw[i] >> 16; f[i*4+2] = txw[i] >> 8; f[i*4+3] = txw[i];
    }
    fw_process(f);
}
static void run(int n) { while (n--) cycle(); }

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } else { printf("ok   "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* The INI motor settings (cfg.motor.*) reach a board that holds others,
 * also when frames land in a settings save. */
static int motor_checks(void)
{
    hal_u32_t *steps = pin("cfg.motor.steps-per-unit");
    hal_u32_t *up = pin("cfg.motor.max-up-velocity");
    hal_u32_t *down = pin("cfg.motor.max-down-velocity");
    hal_u32_t *accel = pin("cfg.motor.acceleration");
    hal_bit_t *dir = pin("cfg.motor.dir-up-polarity");
    hal_u32_t *fb_up = pin("fb.motor.max-up-velocity");
    hal_u32_t *fb_accel = pin("fb.motor.acceleration");
    hal_u32_t *upper = pin("cfg.thc.upper-limit"), *lower = pin("cfg.thc.lower-limit");
    hal_bit_t *idle = pin("thc.prog-is-idle");
    *upper = 200; *lower = 50; *idle = 1;
    fw.sp = 125 * 16; fw.base = fw.sp; fw.delay_ms = 500; fw.upper = 200 * 16; fw.lower = 50 * 16;
    /* the board's stored settings differ from the INI's */
    fw.steps = 1600; fw.up = 78; fw.down = 78; fw.accel = 23; fw.dir = 1;
    *steps = 1600; *up = 150; *down = 150; *accel = 75; *dir = 0;

    /* 1. startup, with a save starting at the THC write (firmware 3.0 always
     * did this): it drops the THC ack and the motor write after it */
    fw.gap_next = 40;
    run(600);
    CHECK(fw.up == 150 && fw.down == 150 && fw.accel == 75 && fw.dir == 0,
          "startup motor settings reach the board (up %u down %u accel %u dir %u)",
          fw.up, fw.down, fw.accel, fw.dir);
    CHECK(*fb_up == 150 && *fb_accel == 75, "fb.motor reads them back (%u %u)",
          (unsigned)*fb_up, (unsigned)*fb_accel);
    CHECK(fw.thc_writes == 2, "the unacknowledged THC write is sent once more (%d)",
          fw.thc_writes);

    /* 2. a live setp of the direction, no other change */
    int m0 = fw.motor_writes;
    *dir = 1; run(600);
    CHECK(fw.dir == 1, "live direction change reaches the board (dir %u)", fw.dir);
    CHECK(fw.motor_writes - m0 == 1, "one motor write for it (%d)", fw.motor_writes - m0);

    /* 3. quiet */
    m0 = fw.motor_writes;
    int t0 = fw.thc_writes;
    run(3000);
    CHECK(fw.motor_writes == m0 && fw.thc_writes == t0,
          "no writes while idle (motor %d, THC %d)", fw.motor_writes - m0, fw.thc_writes - t0);

    /* 4. a change during a slow save (worn flash) */
    fw.busy = 250;
    *up = 140; *down = 140; run(2000);
    CHECK(fw.up == 140 && fw.down == 140 && *fb_up == 140,
          "change during a slow save reaches the board and reads back (fw %u fb %u)",
          fw.up, (unsigned)*fb_up);

    /* 5. a value the firmware replaces does not write (and save) forever */
    m0 = fw.motor_writes;
    *steps = 60000; run(5000);
    CHECK(fw.steps == 10000, "out-of-range steps replaced by the firmware (%u)", fw.steps);
    CHECK(fw.motor_writes - m0 <= 2, "bounded motor writes after a replaced value (%d)",
          fw.motor_writes - m0);

    /* 6. a later valid edit still goes through */
    *steps = 2000; run(600);
    CHECK(fw.steps == 2000, "valid edit after that reaches the board (%u)", fw.steps);

    printf("%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
    return fails ? 1 : 0;
}

int main(int argc, char **argv)
{
    fw.save_gap = 40; fw.save_quiet = 500; fw.quiet = -1;
    bspi_chan = "hm2_test.0.bspi.0";
    if (rtapi_app_main() != 0) { printf("setup failed\n"); return 2; }
    if (argc > 1 && !strcmp(argv[1], "motor"))
        return motor_checks();
    hal_u32_t *sp = pin("cfg.thc.set-point");
    hal_float_t *delay = pin("cfg.thc.delay");
    hal_u32_t *upper = pin("cfg.thc.upper-limit"), *lower = pin("cfg.thc.lower-limit");
    hal_bit_t *aset = pin("cmd.enable-autoset");
    hal_bit_t *idle = pin("thc.prog-is-idle");
    hal_float_t *gcode = pin("thc.gcode-cmd");
    hal_u32_t *target = pin("data.target-volts");
    hal_u32_t *observer_mode = pin("cfg.control.observer-mode");
    hal_u32_t *auto_flags = pin("cfg.control.auto-flags");
    hal_u32_t *slow_delay = pin("cfg.control.slowdown-delay-ms");
    hal_float_t *ff_cap = pin("cfg.control.feedforward-cap");
    hal_bit_t *profile_valid = pin("fb.profile.valid");
    hal_u32_t *profile_crc = pin("fb.profile.crc");
    hal_u32_t *profile_reads = pin("fb.profile.read-sequence");
    hal_bit_t *write_profile = pin("cmd.write-control-profile");
    *upper = 200; *lower = 50; *idle = 1;
    fw.sp = 125 * 16; fw.base = fw.sp; fw.delay_ms = 500; fw.upper = 200 * 16; fw.lower = 50 * 16;
    /* a stored profile (version 9 layout) */
    fw.prof[0] = 3;                    /* observer mode Auto */
    fw.prof[3] = 204;                  /* feedforward cap 0.8 */
    put16(fw.prof + 59, 1000);         /* slowdown delay, ms */
    fw.prof[66] = 1;                   /* Auto flags: kerf hold */

    /* 1. startup */
    run(60);
    CHECK(*sp == 125, "startup echoes the stored preset (pin %u)", (unsigned)*sp);
    CHECK(*profile_valid && *profile_crc == dthc_crc32(fw.prof, DTHC_PROFILE_BYTES),
          "startup reads the profile (valid %d, CRC %08X)", (int)*profile_valid,
          (unsigned)*profile_crc);
    CHECK(*observer_mode == 3 && *auto_flags == 1 && *slow_delay == 1000 &&
          *ff_cap > 0.79 && *ff_cap < 0.81,
          "profile fields land on their pins (mode %u flags %u delay %u cap %.2f)",
          (unsigned)*observer_mode, (unsigned)*auto_flags, (unsigned)*slow_delay,
          (double)*ff_cap);
    CHECK(fw.thc_writes == 1, "startup sends the THC settings once (%d)", fw.thc_writes);
    CHECK(fw.sp == 125 * 16, "startup keeps the preset (fw %u)", fw.sp / 16);

    /* 2. Preset Volts edited on the screen */
    int w0 = fw.thc_writes;
    *sp = 130; run(20);
    CHECK(fw.sp == 130 * 16, "screen preset reaches the board (fw %u)", fw.sp / 16);
    CHECK(fw.thc_writes == w0 + 1, "one write for one edit (%d)", fw.thc_writes - w0);
    CHECK(*sp == 130, "pin keeps the edit (%u)", (unsigned)*sp);

    /* 3. Auto-Set: same preset keeps the learned target, a new one resets it */
    *aset = 1; run(5);
    fw.base = 133 * 16;            /* a learned sample */
    *gcode = 3130; run(20);        /* G-code sends the same preset */
    CHECK(fw.base == 133 * 16, "same preset keeps the learned target (%u)", fw.base / 16);
    *sp = 127; run(20);
    CHECK(fw.sp == 127 * 16 && fw.base == 127 * 16, "new screen preset resets the target (sp %u base %u)", fw.sp / 16, fw.base / 16);
    run(5);
    CHECK(*target == 127, "Sample Volts shows the preset (%u)", (unsigned)*target);
    *gcode = 3140; run(20);
    CHECK(fw.sp == 140 * 16 && *sp == 140, "G-code preset still works (fw %u pin %u)", fw.sp / 16, (unsigned)*sp);

    /* 4. an edit while a config read is in flight, at every offset */
    for (int k = 0; k < 8; k++) {
        *delay = 0.6 + 0.01 * k; /* starts a write, then a read */
        run(k);
        unsigned want = 100 + k;
        *sp = want;
        run(30);
        CHECK(fw.sp == want * 16 && *sp == want && fw.last_written_sp == want * 16,
              "edit %d cycles after another write: fw %u, pin %u, last sent %u",
              k, fw.sp / 16, (unsigned)*sp, fw.last_written_sp / 16);
    }

    /* 5. a value the firmware clamps does not loop */
    w0 = fw.thc_writes;
    *sp = 210; run(100);
    CHECK(fw.sp == 120 * 16 && *sp == 120, "clamped preset echoes 120 (fw %u pin %u)", fw.sp / 16, (unsigned)*sp);
    CHECK(fw.thc_writes - w0 == 1, "no write loop after a clamp (%d writes)", fw.thc_writes - w0);

    /* 6. quiet: nothing sent while nothing changes */
    w0 = fw.thc_writes;
    run(500);
    CHECK(fw.thc_writes == w0, "no writes while idle (%d)", fw.thc_writes - w0);

    /* 7. an edit sent while the board saves: the write is lost */
    w0 = fw.thc_writes;
    fw.busy = 40;
    *sp = 133; run(300);
    CHECK(fw.sp == 133 * 16 && *sp == 133,
          "edit during a save reaches the board (fw %u pin %u)", fw.sp / 16, (unsigned)*sp);
    CHECK(fw.thc_writes - w0 == 1, "sent again once the save ends (%d received)",
          fw.thc_writes - w0);

    /* 8. a save starting with the write: its ack is lost */
    w0 = fw.thc_writes;
    fw.gap_next = 40;
    *sp = 134; run(300);
    CHECK(fw.sp == 134 * 16 && *sp == 134,
          "edit whose ack is lost stays (fw %u pin %u)", fw.sp / 16, (unsigned)*sp);
    CHECK(fw.thc_writes - w0 == 2, "and is sent once more (%d received)", fw.thc_writes - w0);

    /* 9. edits a few cycles apart during a save: the last one wins */
    fw.busy = 60;
    for (int k = 0; k < 6; k++) { *sp = 110 + k; run(3); }
    run(400);
    CHECK(fw.sp == 115 * 16 && *sp == 115,
          "quick edits during a save end at the last (fw %u pin %u)", fw.sp / 16, (unsigned)*sp);

    /* 10. a G-code preset and delay one cycle apart (firmware 3.0 lost the
     * second to the first one's save) */
    *gcode = 3122; run(1);
    *gcode = 512; run(300);
    CHECK(fw.sp == 122 * 16 && fw.delay_ms == 1200,
          "G-code preset and delay both reach the board (sp %u delay %u ms)",
          fw.sp / 16, fw.delay_ms);

    /* 11. a profile edit: staged, committed and read back */
    unsigned reads = *profile_reads;
    *auto_flags = 0;
    *write_profile = 1; run(2); *write_profile = 0;
    run(200);
    CHECK(fw.profile_commits == 1 && fw.prof[66] == 0 && fw.prof[0] == 3 &&
          get16(fw.prof + 59) == 1000,
          "profile write reaches the board (commits %d, flags %u, mode %u, delay %u)",
          fw.profile_commits, fw.prof[66], fw.prof[0], get16(fw.prof + 59));
    CHECK(*profile_valid && *profile_reads == reads + 1 && *auto_flags == 0 &&
          *profile_crc == dthc_crc32(fw.prof, DTHC_PROFILE_BYTES),
          "and reads back (valid %d, reads +%u, flags %u)", (int)*profile_valid,
          (unsigned)(*profile_reads - reads), (unsigned)*auto_flags);

    printf("%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
    return fails ? 1 : 0;
}
