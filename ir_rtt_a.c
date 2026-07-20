// ir_rtt_a.c
// A side: send IR burst on GPIO17, wait for response on GPIO10, measure RTT.
// Compile: gcc -O2 -Wall -pthread -o ir_rtt_a ir_rtt_a.c -lpigpio -lrt -lm
// Run:     sudo ./ir_rtt_a 500

#include <pigpio.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>

#define TX_GPIO 17
#define RX_GPIO 10

// Most 38 kHz IR receiver modules are active-low.
// level == 0 means falling edge / IR detected.
#define ACTIVE_LEVEL 0

#define BURST_US 1200
#define GAP_MS 200
#define DEFAULT_ROUNDS 100

static volatile int running = 1;
static volatile int waiting_for_reply = 0;
static volatile uint32_t send_tick = 0;
static volatile int got_reply = 0;
static volatile uint32_t reply_tick = 0;

static int tx_wave = -1;

static void on_sigint(int sig)
{
    (void)sig;
    running = 0;
}

static uint32_t tick_diff(uint32_t end, uint32_t start)
{
    return end - start;
}

static int build_38khz_burst_wave(int gpio, int burst_us)
{
    const int half_period_us = 13; // approx 38.46 kHz
    int cycles = burst_us / (2 * half_period_us);
    int pulse_count = cycles * 2;

    gpioPulse_t *pulses = calloc((size_t)pulse_count, sizeof(gpioPulse_t));
    if (!pulses) {
        fprintf(stderr, "calloc failed\n");
        return -1;
    }

    for (int i = 0; i < cycles; i++) {
        pulses[2 * i].gpioOn = 1u << gpio;
        pulses[2 * i].gpioOff = 0;
        pulses[2 * i].usDelay = half_period_us;

        pulses[2 * i + 1].gpioOn = 0;
        pulses[2 * i + 1].gpioOff = 1u << gpio;
        pulses[2 * i + 1].usDelay = half_period_us;
    }

    gpioWaveClear();
    gpioWaveAddGeneric(pulse_count, pulses);
    free(pulses);

    int wave_id = gpioWaveCreate();
    if (wave_id < 0) {
        fprintf(stderr, "gpioWaveCreate failed: %d\n", wave_id);
        return -1;
    }

    return wave_id;
}

static void rx_alert(int gpio, int level, uint32_t tick)
{
    (void)gpio;

    if (level != ACTIVE_LEVEL) {
        return;
    }

    if (waiting_for_reply && !got_reply) {
        reply_tick = tick;
        got_reply = 1;
        waiting_for_reply = 0;
    }
}

static void send_ir_burst(void)
{
    send_tick = gpioTick();
    waiting_for_reply = 1;
    got_reply = 0;

    int rc = gpioWaveTxSend(tx_wave, PI_WAVE_MODE_ONE_SHOT);
    if (rc < 0) {
        fprintf(stderr, "gpioWaveTxSend failed: %d\n", rc);
        waiting_for_reply = 0;
    }

    while (gpioWaveTxBusy()) {
        gpioDelay(10);
    }
}

static int compare_uint32(const void *a, const void *b)
{
    uint32_t val_a = *(const uint32_t *)a;
    uint32_t val_b = *(const uint32_t *)b;
    if (val_a < val_b) return -1;
    if (val_a > val_b) return 1;
    return 0;
}

int main(int argc, char **argv)
{
    int rounds = DEFAULT_ROUNDS;

    if (argc >= 2) {
        rounds = atoi(argv[1]);
        if (rounds <= 0) rounds = DEFAULT_ROUNDS;
    }

    signal(SIGINT, on_sigint);

    if (gpioInitialise() < 0) {
        fprintf(stderr, "gpioInitialise failed. Try running with sudo.\n");
        return 1;
    }

    gpioSetMode(TX_GPIO, PI_OUTPUT);
    gpioWrite(TX_GPIO, 0);

    gpioSetMode(RX_GPIO, PI_INPUT);
    gpioSetPullUpDown(RX_GPIO, PI_PUD_UP);

    tx_wave = build_38khz_burst_wave(TX_GPIO, BURST_US);
    if (tx_wave < 0) {
        gpioTerminate();
        return 1;
    }

    int rc = gpioSetAlertFunc(RX_GPIO, rx_alert);
    if (rc < 0) {
        fprintf(stderr, "gpioSetAlertFunc failed: %d\n", rc);
        gpioWaveDelete(tx_wave);
        gpioTerminate();
        return 1;
    }

    printf("A started.\n");
    printf("TX GPIO=%d, RX GPIO=%d, burst=%d us\n", TX_GPIO, RX_GPIO, BURST_US);
    printf("round,rtt_us\n");

    uint32_t *rtts = malloc(rounds * sizeof(uint32_t));
    if (!rtts) {
        fprintf(stderr, "Failed to allocate memory for RTT values\n");
        gpioSetAlertFunc(RX_GPIO, NULL);
        gpioWaveDelete(tx_wave);
        gpioWrite(TX_GPIO, 0);
        gpioTerminate();
        return 1;
    }

    int received = 0;
    uint64_t sum = 0;
    uint32_t min_rtt = UINT32_MAX;
    uint32_t max_rtt = 0;

    for (int i = 1; running && i <= rounds; i++) {
        send_ir_burst();

        int waited_us = 0;
        while (running && !got_reply && waited_us < 50000) {
            gpioDelay(100);
            waited_us += 100;
        }

        if (got_reply) {
            uint32_t rtt = tick_diff(reply_tick, send_tick);
            printf("%d,%u\n", i, rtt);
            fflush(stdout);

            rtts[received] = rtt;
            received++;
            sum += rtt;

            if (rtt < min_rtt) min_rtt = rtt;
            if (rtt > max_rtt) max_rtt = rtt;
        } else {
            printf("%d,timeout\n", i);
            fflush(stdout);
            waiting_for_reply = 0;
        }

        gpioDelay(GAP_MS * 1000);
    }

    if (received > 0) {
        double mean = (double)sum / received;
        double variance_sum = 0.0;
        for (int idx = 0; idx < received; idx++) {
            double diff = rtts[idx] - mean;
            variance_sum += diff * diff;
        }
        double stddev = sqrt(variance_sum / received);

        qsort(rtts, received, sizeof(uint32_t), compare_uint32);

        uint32_t p50 = rtts[(int)(received * 0.50)];
        uint32_t p95 = rtts[(int)(received * 0.95)];
        uint32_t p99 = rtts[(int)(received * 0.99)];

        printf("\nSummary:\n");
        printf("received=%d/%d\n", received, rounds);
        printf("avg_rtt_us=%.2f\n", mean);
        printf("stddev_rtt_us=%.2f\n", stddev);
        printf("min_rtt_us=%u\n", min_rtt);
        printf("p50_rtt_us=%u\n", p50);
        printf("p95_rtt_us=%u\n", p95);
        printf("p99_rtt_us=%u\n", p99);
        printf("max_rtt_us=%u\n", max_rtt);
    }

    free(rtts);

    gpioSetAlertFunc(RX_GPIO, NULL);
    gpioWaveDelete(tx_wave);
    gpioWrite(TX_GPIO, 0);
    gpioTerminate();

    return 0;
}