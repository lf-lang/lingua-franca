// ir_rtt_b.c
// B side: wait for IR burst on GPIO10, immediately reply with IR burst on GPIO17.
// Compile: gcc -O2 -Wall -pthread -o ir_rtt_b ir_rtt_b.c -lpigpio -lrt
// Run:     sudo ./ir_rtt_b

#include <pigpio.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <signal.h>

#define TX_GPIO 17
#define RX_GPIO 10

// Most 38 kHz IR receiver modules are active-low.
#define ACTIVE_LEVEL 0

#define BURST_US 1200
#define LOCKOUT_US 20000

static volatile int running = 1;
static volatile uint32_t last_reply_tick = 0;
static volatile unsigned long reply_count = 0;

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

static void send_reply(void)
{
    if (gpioWaveTxBusy()) {
        return;
    }

    int rc = gpioWaveTxSend(tx_wave, PI_WAVE_MODE_ONE_SHOT);
    if (rc < 0) {
        fprintf(stderr, "gpioWaveTxSend failed: %d\n", rc);
        return;
    }

    reply_count++;
}

static void rx_alert(int gpio, int level, uint32_t tick)
{
    (void)gpio;

    if (level != ACTIVE_LEVEL) {
        return;
    }

    uint32_t since_last = tick_diff(tick, last_reply_tick);

    if (last_reply_tick != 0 && since_last < LOCKOUT_US) {
        return;
    }

    last_reply_tick = tick;
    send_reply();
}

int main(void)
{
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

    printf("B responder started.\n");
    printf("TX GPIO=%d, RX GPIO=%d, burst=%d us\n", TX_GPIO, RX_GPIO, BURST_US);
    printf("Waiting for IR bursts...\n");

    unsigned long last_printed = 0;

    while (running) {
        gpioDelay(500000);

        if (reply_count != last_printed) {
            printf("reply_count=%lu\n", reply_count);
            fflush(stdout);
            last_printed = reply_count;
        }
    }

    gpioSetAlertFunc(RX_GPIO, NULL);
    gpioWaveDelete(tx_wave);
    gpioWrite(TX_GPIO, 0);
    gpioTerminate();

    return 0;
}