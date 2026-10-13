#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printf("Usage: %s <uri_tx>\n", argv[0]);
        printf("Example: %s \"ip:192.168.3.1\"\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char* uri_tx = argv[1];
    const long timeoutUs = 5000000;
    size_t iteration_count = 200;

    printf("TX SDR URI: %s\n", uri_tx);

    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    SoapySDRKwargs_set(&args, "uri", uri_tx);
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "timestamp_every", "1920");
    SoapySDRKwargs_set(&args, "loopback", "0");

    SoapySDRDevice* sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    if (sdr == NULL) {
        printf("SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
        return EXIT_FAILURE;
    }

    const double freq_tx = 433e6;
    const double sample_rate = 1e6;

    SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, sample_rate);
    SoapySDRDevice_setFrequency (sdr, SOAPY_SDR_TX, 0, freq_tx, NULL);
    SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, 0, -40.0);

    printf("TX freq: %.0f Hz, sample rate: %.0f Hz\n",
           SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_TX, 0),
           SoapySDRDevice_getSampleRate(sdr, SOAPY_SDR_TX, 0));

    size_t channels[] = {0};
    SoapySDRStream* txStream = SoapySDRDevice_setupStream(
        sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, 1, NULL);
    if (!txStream) {
        printf("setupStream TX fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }

    size_t tx_mtu = SoapySDRDevice_getStreamMTU(sdr, txStream);
    printf("MTU TX: %zu\n", tx_mtu);

    int16_t* tx_buff = (int16_t*)malloc(2 * tx_mtu * sizeof(int16_t));
    if (!tx_buff) {
        printf("malloc failed\n");
        SoapySDRDevice_closeStream(sdr, txStream);
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
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
    long long buffer_duration_ns = (long long)tx_mtu * 1000000000LL / (long long)sample_rate;

    for (size_t n = 0; n < iteration_count; n++)
    {
        void* tx_buffs[] = {tx_buff};
        int flags = SOAPY_SDR_HAS_TIME; 
        tx_time += buffer_duration_ns;

        int st = SoapySDRDevice_writeStream(sdr, txStream,
                                            (const void* const*)tx_buffs,
                                            tx_mtu, &flags, tx_time, timeoutUs);
        if ((size_t)st != tx_mtu) {
            printf("writeStream error %d: %s\n", st, SoapySDRDevice_lastError());
        } else {
            printf("TX buffer %zu sent: %d samples\n", n, st);
        }
    }

    SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);
    SoapySDRDevice_closeStream(sdr, txStream);
    SoapySDRDevice_unmake(sdr);

    free(tx_buff);

    printf("test complete!\n");
    return EXIT_SUCCESS;
}