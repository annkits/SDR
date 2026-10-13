#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printf("Usage: %s <uri_rx>\n", argv[0]);
        printf("Example: %s \"ip:192.168.2.1\"\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char* uri_rx = argv[1];
    const long timeoutUs = 5000000; 
    long long last_time = 0;
    size_t iteration_count = 200;

    printf("RX SDR URI: %s\n", uri_rx);

    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", uri_rx);
    // SoapySDRKwargs_set(&args, "direct", "1");
    // SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&args, "loopback", "0");

    SoapySDRDevice* sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    if (sdr == NULL) {
        printf("SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
        return EXIT_FAILURE;
    }

    const double freq_rx = 433e6;
    const double sample_rate = 1e6;

    SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, sample_rate);
    SoapySDRDevice_setFrequency (sdr, SOAPY_SDR_RX, 0, freq_rx, NULL);
    SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, 0, 35.0);

    printf("RX freq: %.0f Hz, sample rate: %.0f Hz\n",
           SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_RX, 0),
           SoapySDRDevice_getSampleRate(sdr, SOAPY_SDR_RX, 0));

    size_t channels[] = {0};
    SoapySDRStream* rxStream = SoapySDRDevice_setupStream(
        sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, 1, NULL);
    if (!rxStream) {
        printf("setupStream RX fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }

    size_t rx_mtu = SoapySDRDevice_getStreamMTU(sdr, rxStream);
    printf("MTU RX: %zu\n", rx_mtu);

    int16_t* rx_buffer = (int16_t*)malloc(2 * rx_mtu * sizeof(int16_t));
    if (!rx_buffer) {
        printf("malloc failed\n");
        SoapySDRDevice_closeStream(sdr, rxStream);
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
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
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }

    printf("Start RX...\n");

    for (size_t n = 0; n < iteration_count; n++)
    {
        void* rx_buffs[] = {rx_buffer};
        int flags = 0;
        long long timeNs = 0;

        int sr = SoapySDRDevice_readStream(sdr, rxStream, rx_buffs, rx_mtu,
                                           &flags, &timeNs, timeoutUs);
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
    SoapySDRDevice_unmake(sdr);

    free(rx_buffer);

    printf("Saved received samples to rxdata.pcm\n");
    printf("test complete!\n");
    return EXIT_SUCCESS;
}