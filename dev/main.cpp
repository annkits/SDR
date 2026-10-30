#include <SoapySDR/Device.h>   // инициализация устройства
#include <SoapySDR/Formats.h>  // форматы сэмплов
#include <signal.h>            // SIGINT
#include <stdint.h>
#include <stdio.h>             // printf
#include <stdlib.h>            // malloc, free
#include <string.h>            // strcmp

// Флаг остановки по Ctrl+C
static volatile sig_atomic_t running = 1;
static void on_sigint(int) { running = 0; }

// Частоты каналов: downlink и uplink
static const double FREQ_DOWN = 433e6;
static const double FREQ_UP   = 434e6;

// Параметры BPSK
static const size_t SPS = 10;     // отсчетов на символ (1 МГц / 10 = 100 кбод)
static const int16_t AMP = 1500;  // амплитуда, ЦАП 12 бит (до 2047)

// Преамбула - код Баркера-13
static const uint8_t PREAMBLE[] = {1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1};
static const size_t PREAMBLE_LEN = sizeof(PREAMBLE) / sizeof(PREAMBLE[0]);

// Генератор PRBS-9 (x^9 + x^5 + 1, период 511)
static const uint16_t PRBS9_SEED = 0x1FF;  // начальное состояние, не 0
static uint16_t lfsr = PRBS9_SEED;

static uint8_t prbs9_next(void) {
  uint8_t bit = ((lfsr >> 8) ^ (lfsr >> 4)) & 1;
  lfsr = ((lfsr << 1) | bit) & 0x1FF;
  return bit;
}

// Бит в уровень: 1 -> +AMP, 0 -> -AMP
static int16_t bit_level(uint8_t bit) { return bit ? AMP : -AMP; }

// Свертка y[n] = sum h[m] * x[n - m], хвост после n отсчетов отбрасывается
static void convolve(const int16_t *x, size_t n, const int16_t *h, size_t h_len, int16_t *y) {
  for (size_t i = 0; i < n; i++) {
    int32_t acc = 0;
    for (size_t m = 0; m < h_len && m <= i; m++) acc += h[m] * x[i - m];
    y[i] = (int16_t)acc;
  }
}

// Пакет: преамбула + биты PRBS-9, сигнал только в I.
// Генератор сбрасывается в начале пакета, поэтому все пакеты одинаковые.
// Возвращает количество бит данных.
static size_t fill_packet(int16_t *buf, size_t n_samples) {
  lfsr = PRBS9_SEED;
  size_t n_symbols = n_samples / SPS;
  size_t n_up = n_symbols * SPS;

  // Повышение частоты: символ в первом отсчете, остальные SPS-1 нули
  int16_t *up = (int16_t *)calloc(n_up, sizeof(int16_t));
  for (size_t k = 0; k < n_symbols; k++) {
    uint8_t bit = (k < PREAMBLE_LEN) ? PREAMBLE[k] : prbs9_next();
    up[k * SPS] = bit_level(bit);
  }

  // Формирующий фильтр: прямоугольный импульс из SPS единиц
  int16_t h[SPS];
  for (size_t m = 0; m < SPS; m++) h[m] = 1;
  int16_t *shaped = (int16_t *)malloc(n_up * sizeof(int16_t));
  convolve(up, n_up, h, SPS, shaped);

  // Запись в буфер: I, Q чередуются, сдвиг << 4 под 12-битный ЦАП
  for (size_t n = 0; n < n_up; n++) {
    buf[2 * n] = shaped[n] << 4;  // I
    buf[2 * n + 1] = 0;           // Q
  }
  // Остаток буфера - нули
  for (size_t i = 2 * n_up; i < 2 * n_samples; i++) buf[i] = 0;

  free(up);
  free(shaped);
  return n_symbols - PREAMBLE_LEN;
}

SoapySDRDevice *make_device(const char *uri) {
  SoapySDRKwargs args = {};
  SoapySDRKwargs_set(&args, "driver", "plutosdr");
  SoapySDRKwargs_set(&args, "uri", uri);
  SoapySDRKwargs_set(&args, "direct", "1");              // прямой доступ к буферам
  SoapySDRKwargs_set(&args, "timestamp_every", "1920");  // размер буфера с меткой времени
  SoapySDRKwargs_set(&args, "loopback", "0");            // работа через антенны
  SoapySDRDevice *sdr = SoapySDRDevice_make(&args);
  SoapySDRKwargs_clear(&args);
  if (sdr == NULL) {
    printf("SoapySDRDevice_make fail (%s): %s\n", uri, SoapySDRDevice_lastError());
  }
  return sdr;
}


int run_rx(const char *uri, double freq, const char *filename) {
  SoapySDRDevice *sdr = make_device(uri);
  if (sdr == NULL) return 1;

  int sample_rate = 1e6;
  printf("RX freq: %.0f Hz\n", freq);  // совпадает с частотой TX другой SDR

  // Параметры RX
  SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, sample_rate);
  SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, freq, NULL);

  // Канал RX (в Pluto один, нулевой)
  size_t channels[] = {0};
  SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, *channels, 20.0);  // усиление приемника

  size_t channel_count = sizeof(channels) / sizeof(channels[0]);
  // Поток приема
  SoapySDRStream *rxStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, channel_count, NULL);
  if (rxStream == NULL) {
    printf("setupStream RX fail: %s\n", SoapySDRDevice_lastError());
    SoapySDRDevice_unmake(sdr);
    return 1;
  }

  SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0);  // запуск потока

  // Размер буфера (MTU)
  size_t rx_mtu = SoapySDRDevice_getStreamMTU(sdr, rxStream);
  printf("RX MTU: %zu\n", rx_mtu);

  // Буфер RX
  int16_t *rx_buffer = (int16_t *)malloc(2 * rx_mtu * sizeof(int16_t));

  const long timeoutUs = 400000;
  long long last_time = 0;

  // Файл для сэмплов (перезаписывается)
  FILE *file = fopen(filename, "wb");
  if (file == NULL) {
    printf("Cannot open %s\n", filename);
    free(rx_buffer);
    SoapySDRDevice_closeStream(sdr, rxStream);
    SoapySDRDevice_unmake(sdr);
    return 1;
  }

  size_t buffers_read = 0;

  // Цикл приема
  while (running) {
    void *rx_buffs[] = {rx_buffer};
    int flags;         // флаги приема
    long long timeNs;  // метка времени буфера

    // Чтение буфера RX
    int sr = SoapySDRDevice_readStream(sdr, rxStream, rx_buffs, rx_mtu, &flags, &timeNs, timeoutUs);
    if (sr < 0) {
      printf("readStream error: %i\n", sr);
      continue;
    }

    // Число сэмплов, время и разница с прошлым буфером
    printf("Buffer: %zu - Samples: %i, Flags: %i, Time: %lli, TimeDiff: %lli\n",
           buffers_read, sr, flags, timeNs, timeNs - last_time);
    fwrite(rx_buffer, sizeof(int16_t), 2 * sr, file);
    last_time = timeNs;
    buffers_read++;
  }
  fclose(file);
  free(rx_buffer);
  printf("Saved %zu buffers to %s\n", buffers_read, filename);

  // Остановка и закрытие потока
  SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);
  SoapySDRDevice_closeStream(sdr, rxStream);

  // Освобождение устройства
  SoapySDRDevice_unmake(sdr);
  return 0;
}

int run_tx(const char *uri, double freq_tx, double freq_rx) {
  SoapySDRDevice *sdr = make_device(uri);
  if (sdr == NULL) return 1;

  int sample_rate = 1e6;
  // freq_tx совпадает с частотой RX другой SDR, свой RX нужен только для меток времени
  printf("TX freq: %.0f Hz\n", freq_tx);
  // Параметры TX и RX
  SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, sample_rate);
  SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, sample_rate);
  SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, freq_rx, NULL);
  SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, freq_tx, NULL);
  // Канал RX/TX (в Pluto один, нулевой)
  size_t channels[] = {0};
  // Усиление передатчика 0..89, драйвер переводит в ослабление (gain - 89) дБ
  SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, *channels, 89.0);

  size_t channel_count = sizeof(channels) / sizeof(channels[0]);
  // Потоки: TX для передачи, RX для текущего времени
  SoapySDRStream *txStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, channel_count, NULL);
  SoapySDRStream *rxStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, channel_count, NULL);
  if (txStream == NULL || rxStream == NULL) {
    printf("setupStream fail: %s\n", SoapySDRDevice_lastError());
    SoapySDRDevice_unmake(sdr);
    return 1;
  }

  SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0);  // запуск потока TX
  SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0);  // запуск потока RX

  // Размеры буферов (MTU)
  size_t tx_mtu = SoapySDRDevice_getStreamMTU(sdr, txStream);
  size_t rx_mtu = SoapySDRDevice_getStreamMTU(sdr, rxStream);
  printf("TX MTU: %zu, RX MTU: %zu\n", tx_mtu, rx_mtu);

  // Два буфера TX: пакет BPSK и тишина, передаются по очереди.
  // Метку времени в начало буфера драйвер ставит сам по tx_time.
  int16_t *tx_on = (int16_t *)malloc(2 * tx_mtu * sizeof(int16_t));
  int16_t *tx_off = (int16_t *)calloc(2 * tx_mtu, sizeof(int16_t));
  size_t data_bits = fill_packet(tx_on, tx_mtu);
  printf("BPSK: SPS %zu, symbols per packet %zu (preamble %zu + data %zu)\n",
         SPS, tx_mtu / SPS, PREAMBLE_LEN, data_bits);

  // Буфер RX
  int16_t *rx_buffer = (int16_t *)malloc(2 * rx_mtu * sizeof(int16_t));
  void *rx_buffs[] = {rx_buffer};

  const long timeoutUs = 400000;
  long long last_time = 0;
  size_t buffers_read = 0;

  // Цикл передачи
  while (running) {
    int flags;

    int sr = SoapySDRDevice_readStream(sdr, rxStream, rx_buffs, rx_mtu, &flags, &last_time, timeoutUs);
    if (sr < 0) {
      // без свежего времени буфер ушел бы в прошлое
      printf("readStream error: %i\n", sr);
      continue;
    }

    // Время отправки: на 4 мс позже текущего приема
    long long tx_time = last_time + (4 * 1000 * 1000);

    // Четные буферы - пакет, нечетные - тишина
    void *tx_buffs[] = {(buffers_read % 2 == 0) ? tx_on : tx_off};
    flags = SOAPY_SDR_HAS_TIME;
    int st = SoapySDRDevice_writeStream(sdr, txStream, (const void * const*)tx_buffs, tx_mtu, &flags, tx_time, timeoutUs);
    if ((size_t)st != tx_mtu)
    {
        printf("TX Failed: %i\n", st);
    }

    if (buffers_read % 100 == 0) {
      printf("buffers sent: %zu\n", buffers_read);
    }
    buffers_read++;
  }

  free(tx_on);
  free(tx_off);
  free(rx_buffer);

  // Остановка и закрытие потоков
  SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);
  SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);
  SoapySDRDevice_closeStream(sdr, txStream);
  SoapySDRDevice_closeStream(sdr, rxStream);

  // Освобождение устройства
  SoapySDRDevice_unmake(sdr);
  return 0;
}

int main(int argc, char *argv[]) {
  if (argc < 4) {
    // URI у двух SDR разный (SoapySDRUtil --find)
    printf("Использовать так: %s <rx|tx> <uri> <down|up>\n", argv[0]);
    printf("  down - прямой канал (downlink), %.0f МГц\n", FREQ_DOWN / 1e6);
    printf("  up   - обратный канал (uplink), %.0f МГц\n", FREQ_UP / 1e6);
    printf("Примеры:\n");
    printf("  %s tx usb:1.3.5 down   # SDR №1 передает на %.0f МГц\n", argv[0], FREQ_DOWN / 1e6);
    printf("  %s rx usb:1.5.5 down   # SDR №2 принимает на %.0f МГц\n", argv[0], FREQ_DOWN / 1e6);
    printf("  %s tx usb:1.5.5 up     # SDR №2 передает на %.0f МГц\n", argv[0], FREQ_UP / 1e6);
    printf("  %s rx usb:1.3.5 up     # SDR №1 принимает на %.0f МГц\n", argv[0], FREQ_UP / 1e6);
    return 1;
  }
  const char *uri = argv[2];
  const char *channel = argv[3];

  double freq, freq_other;
  const char *filename;
  if (strcmp(channel, "down") == 0) {
    freq = FREQ_DOWN; freq_other = FREQ_UP; filename = "rx_down.pcm";
  } else if (strcmp(channel, "up") == 0) {
    freq = FREQ_UP; freq_other = FREQ_DOWN; filename = "rx_up.pcm";
  } else {
    printf("Ошибка канала: %s (ожидается down или up)\n", channel);
    return 1;
  }

  signal(SIGINT, on_sigint);
  if (strcmp(argv[1], "rx") == 0){return run_rx(uri, freq, filename);}
  else if (strcmp(argv[1], "tx") == 0) {return run_tx(uri, freq, freq_other);}
  else {
    printf("Ошибка режима работы\n");
    return 1;
  }
}
