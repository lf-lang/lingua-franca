// ir_rtt_UART_a.c
// A side: send 256 bytes of random UART data over IR (modulated at 38kHz),
// wait for B to echo it back, verify correctness, and measure RTT.
// Compile: gcc -O2 -Wall -pthread -o ir_rtt_UART_a ir_rtt_UART_a.c -lpigpio -lrt -lm
// Run:     sudo ./ir_rtt_UART_a [rounds]

#include <pigpio.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <time.h>

#define TX_GPIO 17
#define RX_GPIO 10
#define BAUD_RATE 2400
#define DATA_SIZE 32
#define GAP_MS 500
#define DEFAULT_ROUNDS 10

static volatile int running = 1;

static void on_sigint(int sig)
{
    (void)sig;
    running = 0;
}

// Converts a buffer of bytes into a 38kHz modulated UART wave
static int build_modulated_uart_wave(int gpio, int baud, const uint8_t *buf, int len)
{
    // Max pulses if all bits are 0: len * 10 * 32 = 320 * len.
    int max_pulses = len * 320;
    gpioPulse_t *pulses = calloc((size_t)max_pulses, sizeof(gpioPulse_t));
    if (!pulses) {
        return -1;
    }

    int pulse_idx = 0;
    const int half_period_us = 13; // approx 38.46 kHz
    const int cycles_per_bit = 16; // 16 * 26 = 416 us (~2400 baud)

    for (int i = 0; i < len; i++) {
        uint8_t byte = buf[i];
        // 10 bits: Start bit (0), 8 data bits (LSB first), Stop bit (1)
        for (int bit = 0; bit < 10; bit++) {
            int bit_val;
            if (bit == 0) {
                bit_val = 0; // Start bit
            } else if (bit == 9) {
                bit_val = 1; // Stop bit
            } else {
                bit_val = (byte >> (bit - 1)) & 1; // Data bits LSB first
            }

            if (bit_val == 0) {
                // Modulated 38kHz carrier
                for (int c = 0; c < cycles_per_bit; c++) {
                    pulses[pulse_idx].gpioOn = 1u << gpio;
                    pulses[pulse_idx].gpioOff = 0;
                    pulses[pulse_idx].usDelay = half_period_us;
                    pulse_idx++;

                    pulses[pulse_idx].gpioOn = 0;
                    pulses[pulse_idx].gpioOff = 1u << gpio;
                    pulses[pulse_idx].usDelay = half_period_us;
                    pulse_idx++;
                }
            } else {
                // Idle / Low (No carrier)
                pulses[pulse_idx].gpioOn = 0;
                pulses[pulse_idx].gpioOff = 1u << gpio;
                pulses[pulse_idx].usDelay = cycles_per_bit * 2 * half_period_us; // 416 us
                pulse_idx++;
            }
        }
    }

    gpioWaveClear();
    gpioWaveAddGeneric(pulse_idx, pulses);
    free(pulses);

    int wave_id = gpioWaveCreate();
    return wave_id;
}

// Sends a buffer in chunks of 16 bytes to fit within pigpio wave limits
static int send_modulated_uart(int gpio, int baud, const uint8_t *buf, int len)
{
    const int chunk_size = 32;
    for (int i = 0; i < len; i += chunk_size) {
        int current_chunk = len - i;
        if (current_chunk > chunk_size) {
            current_chunk = chunk_size;
        }

        int wave_id = build_modulated_uart_wave(gpio, baud, buf + i, current_chunk);
        if (wave_id < 0) {
            fprintf(stderr, "Failed to build wave at index %d\n", i);
            return -1;
        }

        int rc = gpioWaveTxSend(wave_id, PI_WAVE_MODE_ONE_SHOT);
        if (rc < 0) {
            fprintf(stderr, "gpioWaveTxSend failed: %d\n", rc);
            gpioWaveDelete(wave_id);
            return -1;
        }

        while (gpioWaveTxBusy()) {
            gpioDelay(500); // check every 0.5ms
        }

        gpioWaveDelete(wave_id);

        // Add a 20ms gap between chunks to allow the receiver's AGC to reset
        if (i + chunk_size < len) {
            gpioDelay(20000); // 20ms
        }
    }
    return 0;
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

    srand(time(NULL));
    signal(SIGINT, on_sigint);

    if (gpioInitialise() < 0) {
        fprintf(stderr, "gpioInitialise failed. Try running with sudo.\n");
        return 1;
    }

    gpioSetMode(TX_GPIO, PI_OUTPUT);
    gpioWrite(TX_GPIO, 0);

    gpioSetMode(RX_GPIO, PI_INPUT);
    gpioSetPullUpDown(RX_GPIO, PI_PUD_UP);

    // Open software serial on RX_GPIO
    int rc = gpioSerialReadOpen(RX_GPIO, BAUD_RATE, 8);
    if (rc < 0) {
        fprintf(stderr, "gpioSerialReadOpen failed: %d\n", rc);
        gpioTerminate();
        return 1;
    }

    printf("A side started. TX GPIO=%d, RX GPIO=%d, baud=%d\n", TX_GPIO, RX_GPIO, BAUD_RATE);
    printf("Preparing to run %d rounds of %d-byte RTT tests...\n\n", rounds, DATA_SIZE);

    uint32_t *rtts = malloc(rounds * sizeof(uint32_t));
    if (!rtts) {
        fprintf(stderr, "Failed to allocate memory for RTT values\n");
        gpioSerialReadClose(RX_GPIO);
        gpioTerminate();
        return 1;
    }

    int received_rounds = 0;
    uint64_t sum_rtt = 0;
    uint32_t min_rtt = UINT32_MAX;
    uint32_t max_rtt = 0;

    uint8_t tx_buf[DATA_SIZE];
    uint8_t rx_buf[DATA_SIZE];

    for (int i = 1; running && i <= rounds; i++) {
        // 1. Generate random data
        for (int j = 0; j < DATA_SIZE; j++) {
            tx_buf[j] = rand() & 0xFF;
        }

        // 2. Clear RX buffer
        char flush_buf[256];
        while (gpioSerialRead(RX_GPIO, flush_buf, sizeof(flush_buf)) > 0) {
            // clear out old data
        }

        printf("Round %d: Sending %d bytes...", i, DATA_SIZE);
        fflush(stdout);

        // 3. Start transmission and measure time
        uint32_t start_tick = gpioTick();

        if (send_modulated_uart(TX_GPIO, BAUD_RATE, tx_buf, DATA_SIZE) < 0) {
            printf(" Send failed.\n");
            continue;
        }
        uint32_t tx_end_tick = gpioTick();

        // 4. Wait for B's echo
        int rx_len = 0;
        uint32_t wait_start = gpioTick();
        int timed_out = 0;

        while (running && rx_len < DATA_SIZE) {
            char chunk[256];
            int read_rc = gpioSerialRead(RX_GPIO, chunk, sizeof(chunk));
            if (read_rc > 0) {
                if (rx_len + read_rc <= DATA_SIZE) {
                    memcpy(rx_buf + rx_len, chunk, (size_t)read_rc);
                    rx_len += read_rc;
                } else {
                    int remaining = DATA_SIZE - rx_len;
                    memcpy(rx_buf + rx_len, chunk, (size_t)remaining);
                    rx_len += remaining;
                    break;
                }
            } else if (read_rc < 0) {
                fprintf(stderr, "\ngpioSerialRead error: %d\n", read_rc);
                break;
            }

            uint32_t elapsed_ms = (gpioTick() - wait_start) / 1000;
            if (elapsed_ms > 4000) { // 4 seconds timeout (accommodates inter-chunk gaps)
                timed_out = 1;
                break;
            }
            gpioDelay(1000); // 1ms sleep
        }
        uint32_t rx_end_tick = gpioTick();

        // 5. Verify correctness
        int match_count = 0;
        for (int j = 0; j < rx_len; j++) {
            if (tx_buf[j] == rx_buf[j]) {
                match_count++;
            }
        }

        // 6. Print result
        if (!timed_out && rx_len == DATA_SIZE && match_count == DATA_SIZE) {
            uint32_t rtt = rx_end_tick - start_tick;
            uint32_t tx_us = tx_end_tick - start_tick;
            uint32_t rx_us = rx_end_tick - tx_end_tick;

            printf(" SUCCESS | RTT: %u us (TX: %u us, RX_wait: %u us)\n", rtt, tx_us, rx_us);
            fflush(stdout);

            rtts[received_rounds] = rtt;
            received_rounds++;
            sum_rtt += rtt;

            if (rtt < min_rtt) min_rtt = rtt;
            if (rtt > max_rtt) max_rtt = rtt;
        } else {
            if (timed_out) {
                printf(" TIMEOUT | Received %d/%d bytes (matched %d)\n", rx_len, DATA_SIZE, match_count);
            } else {
                printf(" FAILED  | Received %d/%d bytes (matched %d)\n", rx_len, DATA_SIZE, match_count);
            }
            fflush(stdout);
        }

        gpioDelay(GAP_MS * 1000);
    }

    // Print summary stats
    if (received_rounds > 0) {
        double mean = (double)sum_rtt / received_rounds;
        double variance_sum = 0.0;
        for (int idx = 0; idx < received_rounds; idx++) {
            double diff = rtts[idx] - mean;
            variance_sum += diff * diff;
        }
        double stddev = sqrt(variance_sum / received_rounds);

        qsort(rtts, (size_t)received_rounds, sizeof(uint32_t), compare_uint32);

        uint32_t p50 = rtts[(int)(received_rounds * 0.50)];
        uint32_t p95 = rtts[(int)(received_rounds * 0.95)];
        uint32_t p99 = rtts[(int)(received_rounds * 0.99)];

        printf("\n--- Summary ---\n");
        printf("Successful Rounds: %d/%d\n", received_rounds, rounds);
        printf("Average RTT:       %.2f us\n", mean);
        printf("Std Dev RTT:       %.2f us\n", stddev);
        printf("Min RTT:           %u us\n", min_rtt);
        printf("p50 RTT:           %u us\n", p50);
        printf("p95 RTT:           %u us\n", p95);
        printf("p99 RTT:           %u us\n", p99);
        printf("Max RTT:           %u us\n", max_rtt);
    } else {
        printf("\nNo successful round completed.\n");
    }

    free(rtts);
    gpioSerialReadClose(RX_GPIO);
    gpioWrite(TX_GPIO, 0);
    gpioTerminate();

    return 0;
}