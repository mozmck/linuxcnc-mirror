/* Host-side simulation of ccc_dthc_bspi THC-config handling (test.sh
 * builds it with the module flags and COMP_C naming the generated C).
 * The driver's generated C is included; HAL, RTAPI and hm2 calls are
 * stubbed.  A fake DTHC6 answers each SPI frame one transfer later, so a
 * reply is parsed two servo cycles after its request. */

/* The firmware this branch's driver works with: 1.x, whose CMD_CONFIG
 * reply is 42 bytes (no response trim). */
#ifndef SIM_FW_REV
#define SIM_FW_REV 10u
#endif
#ifndef SIM_CONFIG_LEN
#define SIM_CONFIG_LEN 42
#endif
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

/* Parameters live in the instance; nothing here reads them back. */
#define PARAM_STUB(fn, T) \
int fn(hal_param_dir_t dir, T *data_addr, int comp_id, const char *fmt, ...) \
{ (void)dir; (void)data_addr; (void)comp_id; (void)fmt; return 0; }
PARAM_STUB(hal_param_u32_newf, hal_u32_t)
PARAM_STUB(hal_param_s32_newf, hal_s32_t)
PARAM_STUB(hal_param_float_newf, hal_float_t)
PARAM_STUB(hal_param_bit_newf, hal_bit_t)

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

static void fw_process(const uint8_t *f)
{
    uint8_t pl[48] = {0};
    if (f[0] != 0x55) { fw_frame(fw.reply, 0, pl, 0); return; }
    uint8_t cmd = f[2];
    const uint8_t *d = f + 3;
    switch (cmd) {
    case DTHC_SPI_CMD_CONFIG:
        pl[0] = 16;
        put16(pl + 1, fw.delay_ms);
        put16(pl + 3, fw.sp);
        put16(pl + 5, fw.upper);
        put16(pl + 7, fw.lower);
        put32(pl + 38, SIM_FW_REV);
        fw_frame(fw.reply, cmd, pl, SIM_CONFIG_LEN);
        break;
    case DTHC_SPI_CMD_CONF_SET:
        if (d[0] == 0) {
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
        fw_frame(fw.reply, 0, pl, 0);
        break;
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

int main(void)
{
    bspi_chan = "hm2_test.0.bspi.0";
    if (rtapi_app_main() != 0) { printf("setup failed\n"); return 2; }
    hal_u32_t *sp = pin("cfg.thc.set-point");
    hal_float_t *delay = pin("cfg.thc.delay");
    hal_u32_t *upper = pin("cfg.thc.upper-limit"), *lower = pin("cfg.thc.lower-limit");
    hal_bit_t *aset = pin("cmd.enable-autoset");
    hal_bit_t *idle = pin("thc.prog-is-idle");
    hal_float_t *gcode = pin("thc.gcode-cmd");
    hal_u32_t *target = pin("data.target-volts");
    *upper = 200; *lower = 50; *idle = 1;
    fw.sp = 125 * 16; fw.base = fw.sp; fw.delay_ms = 500; fw.upper = 200 * 16; fw.lower = 50 * 16;

    /* 1. startup */
    run(60);
    CHECK(*sp == 125, "startup echoes the stored preset (pin %u)", (unsigned)*sp);
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

    printf("%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
    return fails ? 1 : 0;
}
