#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

static SoapySDRDevice* open_pluto(const char* uri)
{
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", uri);
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&args, "loopback", "0");

    SoapySDRDevice* sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    if (sdr == NULL) {
        printf("SoapySDRDevice_make(%s) fail: %s\n", uri, SoapySDRDevice_lastError());
    }
    return sdr;
}

int main(int argc, char* argv[]) {
    const long timeoutUs = 1000000;
    long long last_time = 0;
    size_t iteration_count = 20;

    if (argc < 3) {
        printf("Usage: %s <uri_sdr1_tx> <uri_sdr2_rx>\n", argv[0]);
        printf("Example: %s \"usb:1.2.3\" \"usb:1.4.5\"\n", argv[0]);
        printf("     or: %s \"ip:192.168.2.1\" \"ip:192.168.3.1\"\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char* uri_tx = argv[1];
    const char* uri_rx = argv[2];

    printf("TX SDR URI: %s\n", uri_tx);
    printf("RX SDR URI: %s\n", uri_rx);

    SoapySDRDevice* sdr_tx = NULL;
    SoapySDRDevice* sdr_rx = NULL;
    SoapySDRStream* txStream = NULL;
    SoapySDRStream* rxStream = NULL;
    int16_t* tx_buff = NULL;
    int16_t* rx_buffer = NULL;
    FILE* file = NULL;
    size_t tx_mtu = 0;
    size_t rx_mtu = 0;

    sdr_tx = open_pluto(uri_tx);
    if (!sdr_tx) return EXIT_FAILURE;

    sdr_rx = open_pluto(uri_rx);
    if (!sdr_rx) {
        SoapySDRDevice_unmake(sdr_tx);
        return EXIT_FAILURE;
    }

    const double freq_tx_main = 800e6;
    const double freq_rx_main = 750e6;
    const double sample_rate  = 1e6;

    SoapySDRDevice_setSampleRate(sdr_tx, SOAPY_SDR_TX, 0, sample_rate);
    SoapySDRDevice_setFrequency (sdr_tx, SOAPY_SDR_TX, 0, freq_tx_main, NULL);
    SoapySDRDevice_setSampleRate(sdr_tx, SOAPY_SDR_RX, 0, sample_rate);
    SoapySDRDevice_setFrequency (sdr_tx, SOAPY_SDR_RX, 0, freq_rx_main, NULL);
    SoapySDRDevice_setGain(sdr_tx, SOAPY_SDR_TX, 0, -40.0);
    SoapySDRDevice_setGain(sdr_tx, SOAPY_SDR_RX, 0, 30.0);

    SoapySDRDevice_setSampleRate(sdr_rx, SOAPY_SDR_RX, 0, sample_rate);
    SoapySDRDevice_setFrequency (sdr_rx, SOAPY_SDR_RX, 0, freq_tx_main, NULL);
    SoapySDRDevice_setSampleRate(sdr_rx, SOAPY_SDR_TX, 0, sample_rate);
    SoapySDRDevice_setFrequency (sdr_rx, SOAPY_SDR_TX, 0, freq_rx_main, NULL);
    SoapySDRDevice_setGain(sdr_rx, SOAPY_SDR_RX, 0, 30.0);
    SoapySDRDevice_setGain(sdr_rx, SOAPY_SDR_TX, 0, -40.0);

    printf("SDR1 TX freq: %.0f Hz, RX freq: %.0f Hz\n",
           SoapySDRDevice_getFrequency(sdr_tx, SOAPY_SDR_TX, 0),
           SoapySDRDevice_getFrequency(sdr_tx, SOAPY_SDR_RX, 0));
    printf("SDR2 RX freq: %.0f Hz, TX freq: %.0f Hz\n",
           SoapySDRDevice_getFrequency(sdr_rx, SOAPY_SDR_RX, 0),
           SoapySDRDevice_getFrequency(sdr_rx, SOAPY_SDR_TX, 0));

    size_t channels[] = {0};
    size_t channel_count = 1;

    txStream = SoapySDRDevice_setupStream(
        sdr_tx, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (!txStream) {
        printf("setupStream TX fail: %s\n", SoapySDRDevice_lastError());
        goto cleanup;
    }

    rxStream = SoapySDRDevice_setupStream(
        sdr_rx, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (!rxStream) {
        printf("setupStream RX fail: %s\n", SoapySDRDevice_lastError());
        goto cleanup;
    }

    tx_mtu = SoapySDRDevice_getStreamMTU(sdr_tx, txStream);
    rx_mtu = SoapySDRDevice_getStreamMTU(sdr_rx, rxStream);
    printf("MTU TX: %zu, RX: %zu\n", tx_mtu, rx_mtu);

    tx_buff   = (int16_t*)malloc(2 * tx_mtu * sizeof(int16_t));
    rx_buffer = (int16_t*)malloc(2 * rx_mtu * sizeof(int16_t));
    if (!tx_buff || !rx_buffer) {
        printf("malloc failed\n");
        goto cleanup;
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

    for (size_t i = 0; i < 2; i++) {
        tx_buff[0 + i]  = 0xffff;
        tx_buff[10 + i] = 0xffff;
    }

    SoapySDRDevice_activateStream(sdr_tx, txStream, 0, 0, 0);
    SoapySDRDevice_activateStream(sdr_rx, rxStream, 0, 0, 0);

    printf("Start test...\n");

    file = fopen("rxdata.pcm", "wb");
    if (!file) {
        printf("Cannot open rxdata.pcm\n");
        goto cleanup;
    }

    for (size_t n = 0; n < iteration_count; n++)
    {
        void* rx_buffs[] = {rx_buffer};
        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(sdr_rx, rxStream, rx_buffs, rx_mtu,
                                           &flags, &timeNs, timeoutUs);
        if (sr < 0) {
            printf("readStream error %d: %s\n", sr, SoapySDRDevice_lastError());
            continue;
        }

        printf("Buffer %zu: Samples=%d, Flags=%d, Time=%lld, Diff=%lld\n",
               n, sr, flags, timeNs, timeNs - last_time);
        last_time = timeNs;

        fwrite(rx_buffer, sizeof(int16_t), 2 * rx_mtu, file);
        fflush(file);

        if (n >= 2) {
            long long tx_time = timeNs + (4 * 1000 * 1000);

            for (size_t i = 0; i < 8; i++) {
                uint8_t b = (tx_time >> (i * 8)) & 0xff;
                tx_buff[2 + i] = b << 4;
            }

            void* tx_buffs[] = {tx_buff};
            flags = SOAPY_SDR_HAS_TIME;
            int st = SoapySDRDevice_writeStream(sdr_tx, txStream,
                                                (const void* const*)tx_buffs,
                                                tx_mtu, &flags, tx_time, timeoutUs);
            if ((size_t)st != tx_mtu) {
                printf("TX Failed: %d\n", st);
            } else if (n == 2) {
                printf("First TX sent at time %lld\n", tx_time);
            }
        }
    }

    fclose(file);
    file = NULL;
    printf("Saved received samples to rxdata.pcm\n");

cleanup:
    if (file) fclose(file);
    if (rxStream) {
        SoapySDRDevice_deactivateStream(sdr_rx, rxStream, 0, 0);
        SoapySDRDevice_closeStream(sdr_rx, rxStream);
    }
    if (txStream) {
        SoapySDRDevice_deactivateStream(sdr_tx, txStream, 0, 0);
        SoapySDRDevice_closeStream(sdr_tx, txStream);
    }
    if (sdr_rx) SoapySDRDevice_unmake(sdr_rx);
    if (sdr_tx) SoapySDRDevice_unmake(sdr_tx);
    free(tx_buff);
    free(rx_buffer);

    printf("test complete!\n");
    return EXIT_SUCCESS;
}