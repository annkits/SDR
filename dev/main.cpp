#include <SoapySDR/Device.h>   // инициализация устройства
#include <SoapySDR/Formats.h>  // форматы сэмплов
#include <arpa/inet.h>         // htons, htonl
#include <netinet/in.h>        // sockaddr_in
#include <sys/socket.h>        // socket, sendto
#include <unistd.h>            // close
#include <math.h>              // lround, sqrt
#include <signal.h>            // SIGINT
#include <stdint.h>
#include <stdio.h>             // printf
#include <stdlib.h>            // malloc, free
#include <string.h>            // strcmp, memcpy

// Флаг остановки по Ctrl+C
static volatile sig_atomic_t running = 1;
static void on_sigint(int) { running = 0; }

// Частоты каналов: downlink и uplink
static const double FREQ_DOWN = 433e6;
static const double FREQ_UP   = 434e6;

// UDP-порты на localhost: приемник шлет копию сэмплов для read.py live
static const uint16_t UDP_PORT_DOWN = 5555;
static const uint16_t UDP_PORT_UP   = 5556;
// Отсчетов в датаграмме: 8 + 360 * 4 = 1448 байт, без IP-фрагментации
// (фрагментированные датаграммы на loopback в WSL могут теряться).
// В начале датаграммы uint64 - номер ее первого отсчета с начала приема,
// по нему read.py ставит отсчеты на свое место и видит потери
static const int UDP_CHUNK = 360;

// Управление по UDP: read.py live шлет на управляющий порт команду
// "ключ=значение ..." (пустая - только запрос), процесс применяет ее между
// буферами и отвечает текущими настройками в том же виде.
// Порт: порт данных канала + 10 для rx, + 20 для tx
static const uint16_t CTRL_OFFSET_RX = 10;
static const uint16_t CTRL_OFFSET_TX = 20;

// Параметры модуляции
static const size_t SPS = 10;     // отсчетов на символ (1 МГц / 10 = 100 кбод)
static const int16_t AMP = 1500;      // амплитуда по умолчанию, ЦАП 12 бит
static const int16_t AMP_MAX = 2047;  // предел ЦАП

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

// Бит в уровень: 1 -> +amp, 0 -> -amp
static int16_t bit_level(uint8_t bit, int16_t amp) { return bit ? amp : -amp; }

// Свертка y[n] = sum h[m] * x[n - m], хвост после n отсчетов отбрасывается
static void convolve(const int16_t *x, size_t n, const int16_t *h, size_t h_len, int16_t *y) {
  for (size_t i = 0; i < n; i++) {
    int32_t acc = 0;
    for (size_t m = 0; m < h_len && m <= i; m++) acc += h[m] * x[i - m];
    y[i] = (int16_t)acc;
  }
}

// Пакет: преамбула + биты PRBS-9.
// Преамбула всегда BPSK в I. Данные:
//   BPSK - один бит на символ, только I;
//   QPSK - два бита на символ: первый задает знак I, второй знак Q (код Грея).
// Генератор сбрасывается в начале пакета, поэтому все пакеты одинаковые.
// У QPSK амплитуда amp / sqrt(2) по I и Q: та же мощность, что у BPSK.
// Возвращает количество бит данных.
static size_t fill_packet(int16_t *buf, size_t n_samples, bool qpsk, int16_t amp) {
  lfsr = PRBS9_SEED;
  int16_t amp_qpsk = (int16_t)lround(amp / sqrt(2.0));
  size_t n_symbols = n_samples / SPS;
  size_t n_up = n_symbols * SPS;

  // Повышение частоты: символ в первом отсчете, остальные SPS-1 нули
  int16_t *up_i = (int16_t *)calloc(n_up, sizeof(int16_t));
  int16_t *up_q = (int16_t *)calloc(n_up, sizeof(int16_t));
  for (size_t k = 0; k < n_symbols; k++) {
    if (k < PREAMBLE_LEN) {
      up_i[k * SPS] = bit_level(PREAMBLE[k], amp);
    } else if (qpsk) {
      up_i[k * SPS] = bit_level(prbs9_next(), amp_qpsk);
      up_q[k * SPS] = bit_level(prbs9_next(), amp_qpsk);
    } else {
      up_i[k * SPS] = bit_level(prbs9_next(), amp);
    }
  }

  // Формирующий фильтр: прямоугольный импульс из SPS единиц, отдельно для I и Q
  int16_t h[SPS];
  for (size_t m = 0; m < SPS; m++) h[m] = 1;
  int16_t *shaped_i = (int16_t *)malloc(n_up * sizeof(int16_t));
  int16_t *shaped_q = (int16_t *)malloc(n_up * sizeof(int16_t));
  convolve(up_i, n_up, h, SPS, shaped_i);
  convolve(up_q, n_up, h, SPS, shaped_q);

  // Запись в буфер: I, Q чередуются, сдвиг << 4 под 12-битный ЦАП
  for (size_t n = 0; n < n_up; n++) {
    buf[2 * n] = shaped_i[n] << 4;      // I
    buf[2 * n + 1] = shaped_q[n] << 4;  // Q
  }
  // Остаток буфера - нули
  for (size_t i = 2 * n_up; i < 2 * n_samples; i++) buf[i] = 0;

  free(up_i);
  free(up_q);
  free(shaped_i);
  free(shaped_q);
  return (n_symbols - PREAMBLE_LEN) * (qpsk ? 2 : 1);
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

// ---------------- Управление по UDP ----------------

static int open_control(uint16_t port) {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) return -1;
  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    printf("Control: bind UDP %u fail, управление отключено\n", port);
    close(sock);
    return -1;
  }
  printf("Control: UDP 127.0.0.1:%u\n", port);
  return sock;
}

// Одна команда без ожидания: текст в buf, отправитель в from. false - команд нет
static bool recv_control(int sock, char *buf, size_t size, struct sockaddr_in *from) {
  if (sock < 0) return false;
  socklen_t len = sizeof(*from);
  ssize_t n = recvfrom(sock, buf, size - 1, MSG_DONTWAIT, (struct sockaddr *)from, &len);
  if (n < 0) return false;
  buf[n] = 0;
  return true;
}

// Следующая пара "ключ=значение" из команды: ключ - возвращаемое значение, NULL - пар больше нет
static char *next_pair(char **save, char *cmd, char **value) {
  for (char *tok = strtok_r(cmd, " \n", save); tok; tok = strtok_r(NULL, " \n", save)) {
    char *eq = strchr(tok, '=');
    if (eq == NULL) continue;
    *eq = 0;
    *value = eq + 1;
    return tok;
  }
  return NULL;
}

static void reply_control(int sock, const struct sockaddr_in *to, const char *text) {
  sendto(sock, text, strlen(text), MSG_DONTWAIT, (const struct sockaddr *)to, sizeof(*to));
}

// Команды приемника: freq (Гц), gain (дБ), agc (0/1 - автоматическая регулировка усиления)
static void rx_control(SoapySDRDevice *sdr, int sock) {
  char cmd[512];
  struct sockaddr_in from;
  while (recv_control(sock, cmd, sizeof(cmd), &from)) {
    char *save, *value;
    for (char *key = next_pair(&save, cmd, &value); key; key = next_pair(&save, NULL, &value)) {
      printf("Control: %s = %s\n", key, value);
      if (strcmp(key, "freq") == 0) {
        SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_RX, 0, atof(value), NULL);
      } else if (strcmp(key, "gain") == 0) {
        SoapySDRDevice_setGain(sdr, SOAPY_SDR_RX, 0, atof(value));
      } else if (strcmp(key, "agc") == 0) {
        SoapySDRDevice_setGainMode(sdr, SOAPY_SDR_RX, 0, atoi(value) != 0);
      }
    }
    SoapySDRRange g = SoapySDRDevice_getGainRange(sdr, SOAPY_SDR_RX, 0);
    char reply[256];
    snprintf(reply, sizeof(reply), "role=rx freq=%.0f gain=%.1f gain_min=%.1f gain_max=%.1f agc=%d",
             SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_RX, 0),
             SoapySDRDevice_getGain(sdr, SOAPY_SDR_RX, 0), g.minimum, g.maximum,
             SoapySDRDevice_getGainMode(sdr, SOAPY_SDR_RX, 0) ? 1 : 0);
    reply_control(sock, &from, reply);
  }
}

// Команды передатчика: freq (Гц), gain (дБ), amp (0..AMP_MAX), mod (bpsk/qpsk).
// При смене амплитуды или модуляции пакет tx_on собирается заново
static void tx_control(SoapySDRDevice *sdr, int sock, int16_t *tx_on, size_t tx_mtu,
                       int16_t *amp, bool *qpsk) {
  char cmd[512];
  struct sockaddr_in from;
  while (recv_control(sock, cmd, sizeof(cmd), &from)) {
    bool refill = false;
    char *save, *value;
    for (char *key = next_pair(&save, cmd, &value); key; key = next_pair(&save, NULL, &value)) {
      printf("Control: %s = %s\n", key, value);
      if (strcmp(key, "freq") == 0) {
        SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, atof(value), NULL);
      } else if (strcmp(key, "gain") == 0) {
        SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, 0, atof(value));
      } else if (strcmp(key, "amp") == 0) {
        int a = atoi(value);
        *amp = (int16_t)(a < 0 ? 0 : a > AMP_MAX ? AMP_MAX : a);
        refill = true;
      } else if (strcmp(key, "mod") == 0) {
        if (strcmp(value, "qpsk") == 0 || strcmp(value, "bpsk") == 0) {
          *qpsk = strcmp(value, "qpsk") == 0;
          refill = true;
        }
      }
    }
    if (refill) fill_packet(tx_on, tx_mtu, *qpsk, *amp);
    SoapySDRRange g = SoapySDRDevice_getGainRange(sdr, SOAPY_SDR_TX, 0);
    char reply[256];
    snprintf(reply, sizeof(reply),
             "role=tx freq=%.0f gain=%.1f gain_min=%.1f gain_max=%.1f amp=%d amp_max=%d mod=%s",
             SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_TX, 0),
             SoapySDRDevice_getGain(sdr, SOAPY_SDR_TX, 0), g.minimum, g.maximum,
             *amp, AMP_MAX, *qpsk ? "qpsk" : "bpsk");
    reply_control(sock, &from, reply);
  }
}


int run_rx(const char *uri, double freq, const char *filename, uint16_t port) {
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

  // UDP-сокет для просмотра в реальном времени.
  // Если read.py live не запущен, датаграммы просто теряются, прием не тормозит
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) printf("UDP socket fail, live view disabled\n");
  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  printf("Live: UDP 127.0.0.1:%u\n", port);
  int ctrl = open_control(port + CTRL_OFFSET_RX);

  size_t buffers_read = 0;
  uint64_t samples_read = 0;  // номер первого отсчета текущего буфера
  int16_t dgram[4 + 2 * UDP_CHUNK];

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
    // Копия буфера в UDP кусками по UDP_CHUNK отсчетов, без ожидания
    for (int off = 0; sock >= 0 && off < sr; off += UDP_CHUNK) {
      int n = (sr - off < UDP_CHUNK) ? sr - off : UDP_CHUNK;
      uint64_t pos = samples_read + off;
      memcpy(dgram, &pos, sizeof(pos));
      memcpy(dgram + 4, rx_buffer + 2 * off, 2 * n * sizeof(int16_t));
      sendto(sock, dgram, sizeof(pos) + 2 * n * sizeof(int16_t), MSG_DONTWAIT,
             (struct sockaddr *)&addr, sizeof(addr));
    }
    samples_read += sr;
    last_time = timeNs;
    buffers_read++;
    rx_control(sdr, ctrl);  // команды read.py live, если пришли
  }
  fclose(file);
  if (sock >= 0) close(sock);
  if (ctrl >= 0) close(ctrl);
  free(rx_buffer);
  printf("Saved %zu buffers to %s\n", buffers_read, filename);

  // Остановка и закрытие потока
  SoapySDRDevice_deactivateStream(sdr, rxStream, 0, 0);
  SoapySDRDevice_closeStream(sdr, rxStream);

  // Освобождение устройства
  SoapySDRDevice_unmake(sdr);
  return 0;
}

int run_tx(const char *uri, double freq_tx, double freq_rx, bool qpsk, uint16_t port) {
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

  // Два буфера TX: пакет BPSK/QPSK и тишина, передаются по очереди.
  // Метку времени в начало буфера драйвер ставит сам по tx_time.
  int16_t *tx_on = (int16_t *)malloc(2 * tx_mtu * sizeof(int16_t));
  int16_t *tx_off = (int16_t *)calloc(2 * tx_mtu, sizeof(int16_t));
  int16_t amp = AMP;
  size_t data_bits = fill_packet(tx_on, tx_mtu, qpsk, amp);
  printf("%s: SPS %zu, symbols per packet %zu (preamble %zu + data %zu), data bits %zu\n",
         qpsk ? "QPSK" : "BPSK", SPS, tx_mtu / SPS, PREAMBLE_LEN,
         tx_mtu / SPS - PREAMBLE_LEN, data_bits);

  // Буфер RX
  int16_t *rx_buffer = (int16_t *)malloc(2 * rx_mtu * sizeof(int16_t));
  void *rx_buffs[] = {rx_buffer};

  const long timeoutUs = 400000;
  long long last_time = 0;
  size_t buffers_read = 0;
  int ctrl = open_control(port + CTRL_OFFSET_TX);

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
    tx_control(sdr, ctrl, tx_on, tx_mtu, &amp, &qpsk);  // команды read.py live
  }

  if (ctrl >= 0) close(ctrl);
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
    printf("Использовать так: %s <rx|tx> <uri> <down|up> [bpsk|qpsk]\n", argv[0]);
    printf("  down - прямой канал (downlink), %.0f МГц\n", FREQ_DOWN / 1e6);
    printf("  up   - обратный канал (uplink), %.0f МГц\n", FREQ_UP / 1e6);
    printf("  bpsk|qpsk - модуляция передатчика, по умолчанию bpsk\n");
    printf("Примеры:\n");
    printf("  %s tx usb:1.3.5 down   # SDR №1 передает на %.0f МГц\n", argv[0], FREQ_DOWN / 1e6);
    printf("  %s rx usb:1.5.5 down   # SDR №2 принимает на %.0f МГц\n", argv[0], FREQ_DOWN / 1e6);
    printf("  %s tx usb:1.5.5 up qpsk  # SDR №2 передает QPSK на %.0f МГц\n", argv[0], FREQ_UP / 1e6);
    printf("  %s rx usb:1.3.5 up     # SDR №1 принимает на %.0f МГц\n", argv[0], FREQ_UP / 1e6);
    return 1;
  }
  const char *uri = argv[2];
  const char *channel = argv[3];

  double freq, freq_other;
  const char *filename;
  uint16_t port;
  if (strcmp(channel, "down") == 0) {
    freq = FREQ_DOWN; freq_other = FREQ_UP; filename = "rx_down.pcm"; port = UDP_PORT_DOWN;
  } else if (strcmp(channel, "up") == 0) {
    freq = FREQ_UP; freq_other = FREQ_DOWN; filename = "rx_up.pcm"; port = UDP_PORT_UP;
  } else {
    printf("Ошибка канала: %s (ожидается down или up)\n", channel);
    return 1;
  }

  // Модуляция нужна только передатчику, приемник пишет сырые I/Q
  bool qpsk = false;
  if (argc > 4) {
    if (strcmp(argv[4], "qpsk") == 0) {
      qpsk = true;
    } else if (strcmp(argv[4], "bpsk") != 0) {
      printf("Ошибка модуляции: %s (ожидается bpsk или qpsk)\n", argv[4]);
      return 1;
    }
  }

  signal(SIGINT, on_sigint);
  if (strcmp(argv[1], "rx") == 0){return run_rx(uri, freq, filename, port);}
  else if (strcmp(argv[1], "tx") == 0) {return run_tx(uri, freq, freq_other, qpsk, port);}
  else {
    printf("Ошибка режима работы\n");
    return 1;
  }
}
