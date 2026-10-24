#include <SoapySDR/Device.h>  // Инициализация устройства
#include <SoapySDR/Formats.h>  // Типы данных, используемых для записи сэмплов
#include <signal.h>  // SIGINT
#include <stdint.h>
#include <stdio.h>   //printf
#include <stdlib.h>  //free
#include <string.h>  //strcmp

// Флаг остановки по Ctrl+C, чтобы корректно закрыть файл и потоки
static volatile sig_atomic_t running = 1;
static void on_sigint(int) { running = 0; }

// Частоты каналов: downlink (прямой) и uplink (обратный)
static const double FREQ_DOWN = 433e6;
static const double FREQ_UP   = 434e6;

SoapySDRDevice *make_device(const char *uri) {
  SoapySDRKwargs args = {};
  SoapySDRKwargs_set(&args, "driver","plutosdr");
  SoapySDRKwargs_set(&args, "uri", uri);
  SoapySDRKwargs_set(&args, "direct", "1");  //
  SoapySDRKwargs_set(&args, "timestamp_every","1920");  // Размер буфера + временные метки
  SoapySDRKwargs_set(&args, "loopback", "0");  // Используем антенны или нет
  SoapySDRDevice *sdr = SoapySDRDevice_make(&args);  // Инициализация
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
  printf("RX freq: %.0f Hz\n", freq);  // должна совпадать с частотой TX у передающей SDR

  // Параметры RX части
  SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, sample_rate);
  SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, freq, NULL);

  // Инициализация количества каналов RX (в AdalmPluto он один, нулевой)
  size_t channels[] = {0};
  SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, *channels, 40.0);  // Чувствительность приемника, 0..73

  size_t channel_count = sizeof(channels) / sizeof(channels[0]);
  // Формирование потока для приема сэмплов
  SoapySDRStream *rxStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, channel_count, NULL);
  if (rxStream == NULL) {
    printf("setupStream RX fail: %s\n", SoapySDRDevice_lastError());
    SoapySDRDevice_unmake(sdr);
    return 1;
  }

  SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0);  // start streaming

  // Получение MTU (Maximum Transmission Unit), в нашем случае - размер буферов.
  size_t rx_mtu = SoapySDRDevice_getStreamMTU(sdr, rxStream);
  printf("RX MTU: %zu\n", rx_mtu);

  // Выделяем память под буферы RX
  int16_t *rx_buffer = (int16_t *)malloc(2 * rx_mtu * sizeof(int16_t));

  const long timeoutUs = 400000;
  long long last_time = 0;

  // Создаем файл для записи сэмплов с rx_buff (перезаписываем, а не дописываем)
  FILE *file = fopen(filename, "wb");
  if (file == NULL) {
    printf("Cannot open %s\n", filename);
    free(rx_buffer);
    SoapySDRDevice_closeStream(sdr, rxStream);
    SoapySDRDevice_unmake(sdr);
    return 1;
  }

  size_t buffers_read = 0;

  // Начинается работа с получением сэмплов
  while (running) {
    void *rx_buffs[] = {rx_buffer};
    int flags;         // flags set by receive operation
    long long timeNs;  // timestamp for receive buffer

    // считали буффер RX, записали его в rx_buffer
    int sr = SoapySDRDevice_readStream(sdr, rxStream, rx_buffs, rx_mtu, &flags, &timeNs, timeoutUs);
    if (sr < 0) {
      printf("readStream error: %i\n", sr);
      continue;
    }

    // Смотрим на количество считаных сэмплов, времени прихода и разницы во
    // времени с чтением прошлого буфера
    printf("Buffer: %zu - Samples: %i, Flags: %i, Time: %lli, TimeDiff: %lli\n",
           buffers_read, sr, flags, timeNs, timeNs - last_time);
    fwrite(rx_buffer, sizeof(int16_t), 2 * sr, file);
    last_time = timeNs;
    buffers_read++;
  }
  fclose(file);
  free(rx_buffer);
  printf("Saved %zu buffers to %s\n", buffers_read, filename);

  // stop streaming
  SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);

  // shutdown the stream
  SoapySDRDevice_closeStream(sdr, rxStream);

  // cleanup device handle
  SoapySDRDevice_unmake(sdr);
  return 0;
}

int run_tx(const char *uri, double freq_tx, double freq_rx) {
  SoapySDRDevice *sdr = make_device(uri);
  if (sdr == NULL) return 1;

  int sample_rate = 1e6;
  // freq_tx должна совпадать с частотой RX у принимающей SDR,
  // RX здесь нужен только для временных меток
  printf("TX freq: %.0f Hz\n", freq_tx);
  // Параметры TX части
  SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_RX, 0, sample_rate);
  SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, sample_rate);
  SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, freq_rx, NULL);
  SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, freq_tx, NULL);
  // Инициализация количества каналов RX\\TX (в AdalmPluto он один, нулевой)
  size_t channels[] = {0};
  // Усиление передатчика 0..89, драйвер пересчитывает в ослабление (gain - 89) дБ
  SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, *channels, 89.0);

  size_t channel_count = sizeof(channels) / sizeof(channels[0]);
  // Формирование потоков: TX для передачи, RX для получения текущего времени
  SoapySDRStream *txStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, channel_count, NULL);
  SoapySDRStream *rxStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, channel_count, NULL);
  if (txStream == NULL || rxStream == NULL) {
    printf("setupStream fail: %s\n", SoapySDRDevice_lastError());
    SoapySDRDevice_unmake(sdr);
    return 1;
  }

  SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0);  // start streaming
  SoapySDRDevice_activateStream(sdr, rxStream, 0, 0, 0);  // start streaming

  // Получение MTU (Maximum Transmission Unit), в нашем случае - размер буферов.
  size_t tx_mtu = SoapySDRDevice_getStreamMTU(sdr, txStream);
  size_t rx_mtu = SoapySDRDevice_getStreamMTU(sdr, rxStream);
  printf("TX MTU: %zu, RX MTU: %zu\n", tx_mtu, rx_mtu);

  // Два буфера TX: с сигналом и нулевой (пауза). Чередуя их, получаем
  // прямоугольную огибающую: 1920 отсчетов "включено", 1920 "выключено".
  // Временную метку в начало буфера драйвер вставляет сам по tx_time.
  int16_t *tx_on = (int16_t *)malloc(2 * tx_mtu * sizeof(int16_t));
  int16_t *tx_off = (int16_t *)calloc(2 * tx_mtu, sizeof(int16_t));
  for (size_t i = 0; i < 2 * tx_mtu; i += 2)
  {
    // ЗДЕСЬ БУДУТ ВАШИ СЭМПЛЫ
    tx_on[i] = 1500 << 4;   // I
    tx_on[i+1] = 1500 << 4; // Q
  }

  // Выделяем память под буферы RX
  int16_t *rx_buffer = (int16_t *)malloc(2 * rx_mtu * sizeof(int16_t));
  void *rx_buffs[] = {rx_buffer};

  const long timeoutUs = 400000;
  long long last_time = 0;
  size_t buffers_read = 0;

  // Начинается работа с отправкой сэмплов
  while (running) {
    int flags;

    int sr = SoapySDRDevice_readStream(sdr, rxStream, rx_buffs, rx_mtu, &flags, &last_time, timeoutUs);
    if (sr < 0) {
      // Без свежего времени буфер ушел бы "в прошлое" и был бы отброшен
      printf("readStream error: %i\n", sr);
      continue;
    }

    // Переменная для времени отправки сэмплов относительно текущего приема
    long long tx_time = last_time + (4 * 1000 * 1000);  // на 4 [мс] в будущее

    // Здесь отправляем наш буфер: четные - сигнал, нечетные - тишина
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

  // stop streaming
  SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);
  SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);

  // shutdown the stream
  SoapySDRDevice_closeStream(sdr, txStream);
  SoapySDRDevice_closeStream(sdr, rxStream);

  // cleanup device handle
  SoapySDRDevice_unmake(sdr);
  return 0;
}

int main(int argc, char *argv[]) {
  if (argc < 4) {
    // URI обязателен: у двух SDR он разный (см. SoapySDRUtil --find)
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
