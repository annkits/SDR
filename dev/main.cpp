#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

static const double FREQ_DOWN = 433e6;
static const double FREQ_UP   = 434e6;

static const double SAMPLE_RATE = 1e6;
static const double GAIN_TX = -40.0;
static const double GAIN_RX = 35.0;

static const long TIMEOUT_US = 5000000;
static const size_t ITERATION_COUNT = 200;

static double select_frequency(const char* channel)
{
    if (strcmp(channel, "up") == 0) {
        return FREQ_UP;
    }
    return FREQ_DOWN;
}

static void run_tx(SoapySDRDevice* sdr, double freq)
{
    SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, SAMPLE_RATE);
    SoapySDRDevice_setFrequency (sdr, SOAPY_SDR_TX, 0, freq, NULL);
    SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, 0, GAIN_TX);

    printf("TX freq: %.0f Hz, sample rate: %.0f Hz\n",
           SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_TX, 0),
           SoapySDRDevice_getSampleRate(sdr, SOAPY_SDR_TX, 0));

    size_t channels[] = {0};
    SoapySDRStream* txStream = SoapySDRDevice_setupStream(
        sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, 1, NULL);
    if (!txStream) {
        printf("setupStream TX fail: %s\n", SoapySDRDevice_lastError());
        return;
    }

    size_t tx_mtu = SoapySDRDevice_getStreamMTU(sdr, txStream);
    printf("MTU TX: %zu\n", tx_mtu);

    int16_t* tx_buff = (int16_t*)malloc(2 * tx_mtu * sizeof(int16_t));
    if (!tx_buff) {
        printf("malloc failed\n");
        SoapySDRDevice_closeStream(sdr, txStream);
        return;
    }

    for (size_t i = 0; i < 2 * tx_mtu; i += 2) {
        if ((i / 2) % 2 == 0) {
            tx_buff[i]     = 1500 << 4;
            tx_buff[i + 1] = 1500 << 4;
        } else {
            tx_buff[i]     = 0;
            tx_buff[i + 1] = 0;
        }
    }

    int ret_tx = SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0);
    printf("activateStream TX: %d\n", ret_tx);
    if (ret_tx != 0) {
        printf("activateStream TX failed: %s\n", SoapySDRDevice_lastError());
    }

    printf("Start TX...\n");

    long long tx_time = 0;
    long long buffer_duration_ns =
        (long long)tx_mtu * 1000000000LL / (long long)SAMPLE_RATE;

    for (size_t n = 0; n < ITERATION_COUNT; n++)
    {
        void* tx_buffs[] = {tx_buff};
        int flags = SOAPY_SDR_HAS_TIME;
        tx_time += buffer_duration_ns;

        int st = SoapySDRDevice_writeStream(sdr, txStream,
                                            (const void* const*)tx_buffs,
                                            tx_mtu, &flags, tx_time, TIMEOUT_US);
        if ((size_t)st != tx_mtu) {
            printf("writeStream error %d: %s\n", st, SoapySDRDevice_lastError());
        } else {
            printf("TX buffer %zu sent: %d samples\n", n, st);
        }
    }

    SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);
    SoapySDRDevice_closeStream(sdr, txStream);
    free(tx_buff);

    printf("TX complete!\n");
}

static void run_rx(SoapySDRDevice* sdr, double freq)
{
    SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, SAMPLE_RATE);
    SoapySDRDevice_setFrequency (sdr, SOAPY_SDR_RX, 0, freq, NULL);
    SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, 0, GAIN_RX);

    printf("RX freq: %.0f Hz, sample rate: %.0f Hz\n",
           SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_RX, 0),
           SoapySDRDevice_getSampleRate(sdr, SOAPY_SDR_RX, 0));

    size_t channels[] = {0};
    SoapySDRStream* rxStream = SoapySDRDevice_setupStream(
        sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, 1, NULL);
    if (!rxStream) {
        printf("setupStream RX fail: %s\n", SoapySDRDevice_lastError());
        return;
    }

    size_t rx_mtu = SoapySDRDevice_getStreamMTU(sdr, rxStream);
    printf("MTU RX: %zu\n", rx_mtu);

    int16_t* rx_buffer = (int16_t*)malloc(2 * rx_mtu * sizeof(int16_t));
    if (!rx_buffer) {
        printf("malloc failed\n");
        SoapySDRDevice_closeStream(sdr, rxStream);
        return;
    }

    int ret_rx = SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0);
    printf("activateStream RX: %d\n", ret_rx);
    if (ret_rx != 0) {
        printf("activateStream RX failed: %s\n", SoapySDRDevice_lastError());
    }

    FILE* file = fopen("rxdata.pcm", "wb");
    if (!file) {
        printf("Cannot open rxdata.pcm\n");
        free(rx_buffer);
        SoapySDRDevice_closeStream(sdr, rxStream);
        return;
    }

    printf("Start RX...\n");

    long long last_time = 0;

    for (size_t n = 0; n < ITERATION_COUNT; n++)
    {
        void* rx_buffs[] = {rx_buffer};
        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(sdr, rxStream, rx_buffs, rx_mtu,
                                           &flags, &timeNs, TIMEOUT_US);
        if (sr < 0) {
            printf("readStream error %d: %s\n", sr, SoapySDRDevice_lastError());
            continue;
        }

        printf("Buffer %zu: Samples=%d, Flags=%d, Time=%lld, Diff=%lld\n",
               n, sr, flags, timeNs, timeNs - last_time);
        last_time = timeNs;

        fwrite(rx_buffer, sizeof(int16_t), 2 * sr, file);
    }

    fclose(file);

    SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);
    SoapySDRDevice_closeStream(sdr, rxStream);
    free(rx_buffer);

    printf("Saved received samples to rxdata.pcm\n");
    printf("RX complete!\n");
}

int main(int argc, char* argv[])
{
    if (argc < 4) {
        printf("Usage: %s <uri> <mode> <channel>\n", argv[0]);
        printf("  <uri>     — URI устройства, например: usb:1.3.5 или ip:192.168.2.1\n");
        printf("  <mode>    — режим работы: tx или rx\n");
        printf("  <channel> — канал: down (прямой) или up (обратный)\n");
        printf("Examples:\n");
        printf("  %s usb:1.3.5 tx down   # SDR №1 передаёт на 433 МГц\n", argv[0]);
        printf("  %s usb:1.5.5 rx down   # SDR №2 принимает на 433 МГц\n", argv[0]);
        printf("  %s usb:1.5.5 tx up     # SDR №2 передаёт на 434 МГц\n", argv[0]);
        printf("  %s usb:1.3.5 rx up     # SDR №1 принимает на 434 МГц\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char* uri     = argv[1];
    const char* mode    = argv[2];
    const char* channel = argv[3];

    printf("SDR URI:  %s\n", uri);
    printf("Mode:     %s\n", mode);
    printf("Channel:  %s\n", channel);

    double freq = select_frequency(channel);
    printf("Selected frequency: %.0f Hz\n", freq);

    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", uri);
    SoapySDRKwargs_set(&args, "loopback", "0");

    if (strcmp(mode, "tx") == 0) {
        SoapySDRKwargs_set(&args, "direct", "1");
        SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    }

    SoapySDRDevice* sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    if (sdr == NULL) {
        printf("SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
        return EXIT_FAILURE;
    }

    if (strcmp(mode, "tx") == 0) {
        run_tx(sdr, freq);
    } else if (strcmp(mode, "rx") == 0) {
        run_rx(sdr, freq);
    } else {
        printf("Unknown mode: %s (expected 'tx' or 'rx')\n", mode);
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }

    SoapySDRDevice_unmake(sdr);

    printf("test complete!\n");
    return EXIT_SUCCESS;
}