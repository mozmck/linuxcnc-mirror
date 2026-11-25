/* Host-side simulation of ccc_dthc_bspi receive alignment (test.sh builds it
 * with the module flags and COMP_C naming the generated C).  The driver's
 * generated C is included with HAL, RTAPI and hm2 calls stubbed, and runs
 * against:
 *
 *  - the EBOB BSPI (firmware/ebob1.1/bufferedspi.vhd): 16-word SRL FIFOs
 *    whose count saturates at 16, so a push into a full one drops the
 *    oldest word and a pop from an empty one returns stale slot 15 and
 *    changes nothing; one 32-bit CS frame at a time at 1 MHz; the FIFO
 *    clear, which also stops the word being shifted;
 *  - hm2_eth at a 1 ms servo period: the read request pops the twelve TRAM
 *    words 25 us into the cycle; a read timeout (request or reply lost)
 *    skips the driver's read() and sends the write after the 800 us
 *    deadline; a write packet carries any clear ahead of the twelve pushes;
 *  - a byte-level DTHC6 (spi_comm.cpp): 48-byte DMA frames, a frame whose
 *    byte count stops for the partial timeout restarts from byte 0, and a
 *    bad request leaves the previous reply in place.
 *
 * Every Ethernet fault and servo-latency case must end with good frames on
 * every cycle and comm.online true. */
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
static int clear_queued;    /* hm2_bspi_clear_fifo() since the last write packet */
int hm2_bspi_setup_chan(char *n, int c, int cs, int b, double m, int d, int cp, int ph, int cl, int e, int s)
{ (void)n; (void)c; (void)cs; (void)b; (void)m; (void)d; (void)cp; (void)ph; (void)cl; (void)e; (void)s; return 0; }
int hm2_bspi_set_read_function(char *n, int (*f)(void *), void *s) { (void)n; drv_read = f; drv_sub = s; return 0; }
int hm2_bspi_set_write_function(char *n, int (*f)(void *), void *s) { (void)n; drv_write = f; drv_sub = s; return 0; }
int hm2_allocate_bspi_tram(char *n) { (void)n; return 0; }
int hm2_tram_add_bspi_frame(char *n, int c, rtapi_u32 **w, rtapi_u32 **r)
{ (void)n; (void)c; *w = &txw[tram_i]; *r = &rxw[tram_i]; tram_i++; return 0; }
int hm2_bspi_clear_fifo(char *n) { (void)n; clear_queued = 1; return 0; }

#include COMP_C

/* ---- timing, us ---- */
#define PERIOD_US    1000.0
#define NET_US       25.0     /* host <-> EBOB, one way */
#define TIMEOUT_US   800.0    /* packet-read-timeout 80 % */
#define POST_US      40.0     /* rest of the servo thread after hm2.read */
#define BYTE_US      8.0      /* 1 MHz */
#define WORD_GAP_US  0.05     /* start/CS/Dav clocks at 100 MHz */

static double thread_us = 250.0;     /* read request -> write packet */
static double partial_us = 200.0;    /* DTHC6 partial-frame timeout */

/* ---- EBOB BSPI FIFOs, as bufferedspi.vhd ---- */
typedef struct { uint32_t srl[16]; double t[16]; int count; } fifo_t;
static fifo_t ofifo, ififo;

static void fifo_push(fifo_t *f, uint32_t w, double t)
{
    memmove(f->srl + 1, f->srl, 15 * sizeof f->srl[0]);
    memmove(f->t + 1, f->t, 15 * sizeof f->t[0]);
    f->srl[0] = w;
    f->t[0] = t;
    if (f->count < 16) f->count++;
}
static uint32_t fifo_pop(fifo_t *f)
{
    uint32_t w = f->srl[f->count ? f->count - 1 : 15];
    if (f->count) f->count--;
    return w;
}

/* ---- DTHC6 ---- */
static struct {
    uint8_t rx[48], tx[48];
    int rxn, txn;
    double last;
    uint32_t seq;
    int restarts;
} d6;

static void d6_reply_idle(void)
{
    memset(d6.tx, 0, 48);
    d6.tx[0] = 0x55; d6.tx[1] = 3; d6.tx[2] = 0;
    d6.tx[3] = dthc_crc8(d6.tx, 3);
}

/* spi_process_packet(): a bad request keeps the old reply.  Every good one
 * gets a full-length reply (CRC at byte 47) carrying a sequence number. */
static void d6_process(void)
{
    uint8_t n = d6.rx[1];
    if (d6.rx[0] != 0x55 || n < 3 || n >= 48 || d6.rx[n] != dthc_crc8(d6.rx, n))
        return;
    d6.seq++;
    memset(d6.tx, 0, 48);
    d6.tx[0] = 0x55; d6.tx[1] = 47; d6.tx[2] = 0;
    for (int i = 3; i < 47; i++)
        d6.tx[i] = (uint8_t)(d6.seq + i);
    d6.tx[47] = dthc_crc8(d6.tx, 47);
}

static uint8_t d6_exchange(double t, uint8_t mosi)
{
    /* spi_comm_service_100us(): a frame part-way through restarts */
    if (d6.rxn && t - d6.last >= partial_us) {
        d6.rxn = 0;
        d6.txn = 0;
        d6.restarts++;
    }
    uint8_t miso = (d6.txn < 48) ? d6.tx[d6.txn] : 0;
    d6.txn++;
    d6.rx[d6.rxn++] = mosi;
    d6.last = t;
    if (d6.rxn == 48) {         /* DMA done: process and re-arm */
        d6_process();
        d6.rxn = 0;
        d6.txn = 0;
    }
    return miso;
}

/* ---- EBOB BSPI shifter ---- */
static struct { int active, bytes; double start, busy_until; uint32_t mosi, miso; } sh;

static void bspi_advance(double t)
{
    for (;;) {
        if (sh.active) {
            double tb = sh.start + (sh.bytes + 1) * BYTE_US;
            if (tb > t) return;
            uint8_t mi = d6_exchange(tb, (uint8_t)(sh.mosi >> (24 - 8 * sh.bytes)));
            sh.miso = (sh.miso << 8) | mi;
            if (++sh.bytes == 4) {
                fifo_push(&ififo, sh.miso, tb);
                sh.active = 0;
                sh.busy_until = tb + WORD_GAP_US;
            }
            continue;
        }
        if (!ofifo.count) return;
        double start = ofifo.t[ofifo.count - 1];
        if (start < sh.busy_until) start = sh.busy_until;
        if (start > t) return;
        sh.mosi = fifo_pop(&ofifo);
        sh.miso = 0;
        sh.bytes = 0;
        sh.start = start;
        sh.active = 1;
    }
}

/* The clear drops the word being shifted: the DTHC6 keeps the bytes it had
 * (its peripheral discards a partial byte at CS). */
static void bspi_clear(double t)
{
    bspi_advance(t);
    sh.active = 0;
    sh.busy_until = t;
    ofifo.count = 0;
    ififo.count = 0;
}

/* ---- one servo cycle ---- */
enum { EV_NONE, EV_REQ_LOST, EV_REPLY_LOST, EV_WRITE_LOST, EV_LATE_WRITE };
static long cyc;

static void cycle(int ev, double late_us)
{
    double t0 = cyc++ * PERIOD_US;
    double tr = t0 + NET_US;
    bspi_advance(tr);
    if (ev != EV_REQ_LOST)
        for (int i = 0; i < 12; i++)
            rxw[i] = fifo_pop(&ififo);
    double tw;
    if (ev == EV_REQ_LOST || ev == EV_REPLY_LOST) {
        tw = t0 + TIMEOUT_US + POST_US;     /* hm2_read returned -EAGAIN */
    } else {
        drv_read(drv_sub);
        tw = t0 + thread_us + (ev == EV_LATE_WRITE ? late_us : 0.0);
    }
    clear_queued = 0;
    drv_write(drv_sub);
    double tx = tw + NET_US;
    bspi_advance(tx);
    if (ev != EV_WRITE_LOST) {
        if (clear_queued) bspi_clear(tx);
        for (int i = 0; i < 12; i++)
            fifo_push(&ofifo, txw[i], tx);
        bspi_advance(tx);
    }
}
static void run(int n) { while (n--) cycle(EV_NONE, 0.0); }

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } else { printf("ok   "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static hal_bit_t *online;
static hal_u32_t *good, *bad, *resyncs;

/* One fault (or a short sequence), then the link must carry a good frame on
 * every one of the last 50 cycles. */
static void scenario(const char *name, const int *evs, const double *late, int nev)
{
    run(50);
    unsigned g0 = *good, b0 = *bad, r0 = *resyncs;
    int d0 = d6.restarts;
    for (int i = 0; i < nev; i++)
        cycle(evs[i], late ? late[i] : 0.0);
    run(50);
    unsigned g1 = *good, b1 = *bad;
    run(50);
    CHECK(*online && *good - g1 == 50 && *bad == b1,
          "%-34s thread %3.0f us, timeout %3.0f us: %u bad frames, %u clears, %d DTHC6 restarts",
          name, thread_us, partial_us, b1 - b0, *resyncs - r0, d6.restarts - d0);
    CHECK(b1 - b0 <= (unsigned)(3 * nev + 2), "%-34s bad frames bounded (%u)", name, b1 - b0);
    (void)g0;
}

int main(void)
{
    bspi_chan = "hm2_test.0.bspi.0";
    if (rtapi_app_main() != 0) { printf("setup failed\n"); return 2; }
    online = pin("comm.online");
    good = pin("comm.good-frames");
    bad = pin("comm.bad-frames");
    resyncs = pin("comm.rx-resyncs");
    d6_reply_idle();
    clear_queued = 0;

    /* steady link: no clears once it is up */
    run(200);
    unsigned r0 = *resyncs, b0 = *bad;
    run(2000);
    CHECK(*online && *bad == b0 && *resyncs == r0,
          "steady link: no bad frames and no clears in 2000 cycles (%u, %u)",
          *bad - b0, *resyncs - r0);

    static const double threads[] = { 100.0, 180.0, 250.0, 400.0 };
    static const double partials[] = { 100.0, 200.0 };
    for (unsigned p = 0; p < sizeof partials / sizeof partials[0]; p++) {
        partial_us = partials[p];
        for (unsigned t = 0; t < sizeof threads / sizeof threads[0]; t++) {
            thread_us = threads[t];
            int e;
            e = EV_REQ_LOST;   scenario("read request lost", &e, 0, 1);
            e = EV_REPLY_LOST; scenario("read reply lost or late", &e, 0, 1);
            e = EV_WRITE_LOST; scenario("write packet lost", &e, 0, 1);
            double l300 = 300.0, l500 = 500.0, l700 = 700.0;
            e = EV_LATE_WRITE; scenario("servo latency, write +300 us", &e, &l300, 1);
            e = EV_LATE_WRITE; scenario("servo latency, write +500 us", &e, &l500, 1);
            e = EV_LATE_WRITE; scenario("servo latency, write +700 us", &e, &l700, 1);
            int two[] = { EV_REPLY_LOST, EV_REPLY_LOST };
            scenario("two read timeouts in a row", two, 0, 2);
            int mix[] = { EV_REPLY_LOST, EV_NONE, EV_REQ_LOST, EV_WRITE_LOST };
            scenario("timeout, good, timeout, write lost", mix, 0, 4);
        }
    }

    /* a noisy minute: 1 % of cycles hit by a random fault */
    partial_us = 200.0;
    thread_us = 250.0;
    uint32_t lcg = 12345u;
    int evs[2000];
    double lates[2000];
    for (int i = 0; i < 2000; i++) {
        lcg = lcg * 1103515245u + 12345u;
        unsigned r = (lcg >> 16) % 400u;
        evs[i] = (r < 4) ? (int)(r + 1) : EV_NONE;
        lates[i] = 200.0 + (double)((lcg >> 8) % 600u);
    }
    run(50);
    unsigned bb = *bad;
    for (int i = 0; i < 2000; i++)
        cycle(evs[i], lates[i]);
    run(50);
    unsigned b1 = *bad, g1 = *good;
    run(100);
    CHECK(*online && *bad == b1 && *good - g1 == 100,
          "random faults over 2000 cycles: recovered (%u bad frames during)", b1 - bb);

    printf("%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
    return fails ? 1 : 0;
}
