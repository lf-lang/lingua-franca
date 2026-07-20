// ir_rtt_HK_b.c
// B side (Prover): Hancke-Kuhn Distance Bounding Protocol.
// Waits for challenges, decodes pulse width, looks up shared secret, immediately replies.
// Adapts number of rounds based on incoming sync pulse duration.
// Compile: gcc -O2 -Wall -pthread -o ir_rtt_HK_b ir_rtt_HK_b.c -lpigpio -lrt -lm
// Run:     sudo ./ir_rtt_HK_b

#include <pigpio.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#define TX_GPIO 17
#define RX_GPIO 10
#define ACTIVE_LEVEL 0

// Shared 32-byte secret (256 bits)
static const uint8_t secret[32] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00
};

static volatile int running = 1;

static void on_sigint(int sig)
{
    (void)sig;
    running = 0;
}

// Pre-created waves
static int wave_short = -1;
static int wave_long = -1;

// Builds a 38kHz carrier pulse wave of specified duration (us)
static int build_38khz_burst_wave(int gpio, int burst_us)
{
    const int half_period_us = 13; // approx 38.46 kHz
    int cycles = burst_us / (2 * half_period_us);
    int pulse_count = cycles * 2;

    gpioPulse_t *pulses = calloc((size_t)pulse_count, sizeof(gpioPulse_t));
    if (!pulses) return -1;

    for (int i = 0; i < cycles; i++) {
        pulses[2 * i].gpioOn = 1u << gpio;
        pulses[2 * i].gpioOff = 0;
        pulses[2 * i].usDelay = half_period_us;

        pulses[2 * i + 1].gpioOn = 0;
        pulses[2 * i + 1].gpioOff = 1u << gpio;
        pulses[2 * i + 1].usDelay = half_period_us;
    }

    gpioWaveAddGeneric(pulse_count, pulses);
    free(pulses);

    int wave_id = gpioWaveCreate();
    return wave_id;
}

static int get_bit(const uint8_t *reg, int bit_idx)
{
    int byte_idx = bit_idx / 8;
    int bit_pos = bit_idx % 8;
    return (reg[byte_idx] >> (7 - bit_pos)) & 1;
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

    // Clean up any waves from previous runs
    gpioWaveClear();

    // Pre-build waves
    wave_short = build_38khz_burst_wave(TX_GPIO, 300); // 300us (Bit 0)
    wave_long  = build_38khz_burst_wave(TX_GPIO, 600); // 600us (Bit 1)

    if (wave_short < 0 || wave_long < 0) {
        fprintf(stderr, "Failed to create waves\n");
        gpioTerminate();
        return 1;
    }

    // Split 32-byte secret into two 16-byte registers
    const uint8_t *R0 = secret;
    const uint8_t *R1 = secret + 16;

    printf("B (Prover) started. TX GPIO=%d, RX GPIO=%d\n", TX_GPIO, RX_GPIO);
    printf("Waiting for Sync pulse or fast challenges...\n");

    int fast_phase = 0;
    int round_idx = 0;
    int expected_rounds = 128;

    while (running) {
        // Wait for RX pin to go LOW (start of pulse)
        while (running && gpioRead(RX_GPIO) != ACTIVE_LEVEL) {
            gpioDelay(100); // 100us sleep to save CPU when idle
        }
        
        if (!running) break;

        uint32_t rx_start_tick = gpioTick();

        // Wait for RX pin to go HIGH (end of pulse)
        while (gpioRead(RX_GPIO) == ACTIVE_LEVEL) {
            // Tight loop for high timing precision during transmission
        }
        uint32_t rx_end_tick = gpioTick();

        uint32_t pulse_width = rx_end_tick - rx_start_tick;

        // Check for Sync pulse (duration > 1500us)
        if (pulse_width > 1500) {
            int num_rounds = 128;
            if (pulse_width < 2500) {
                num_rounds = 32;
            } else if (pulse_width < 3500) {
                num_rounds = 64;
            } else {
                num_rounds = 128;
            }

            printf("\n--- New Session Started ---\n");
            printf("Sync pulse detected (width: %u us). Set expected rounds to %d.\n", pulse_width, num_rounds);
            fflush(stdout);

            round_idx = 0;
            expected_rounds = num_rounds;
            fast_phase = 1;
            continue;
        }

        // If not in fast phase, ignore short/long pulses
        if (!fast_phase) {
            continue;
        }

        // Decode challenge bit
        int challenge = (pulse_width < 450) ? 0 : 1;

        // Lookup response bit from R0 or R1
        int response = get_bit((challenge == 0) ? R0 : R1, round_idx);

        // Send response immediately
        int tx_wave = (response == 0) ? wave_short : wave_long;
        gpioWaveTxSend(tx_wave, PI_WAVE_MODE_ONE_SHOT);

        // Wait for transmission to end
        while (gpioWaveTxBusy()) {
            // Tight loop
        }

        round_idx++;
        if (round_idx >= expected_rounds) {
            printf("Completed %d rounds of fast exchange. Going back to idle.\n", expected_rounds);
            fflush(stdout);
            fast_phase = 0;
        }
    }

    gpioWrite(TX_GPIO, 0);
    gpioTerminate();
    return 0;
}
