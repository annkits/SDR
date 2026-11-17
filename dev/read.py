import os
import socket
import sys
import threading
import time

import numpy as np
from imgui_bundle import imgui, immapp, implot

FS = 1e6                  # частота дискретизации, как в main.cpp
BUFFER = 1920             # отсчетов в пакете
PREAMBLE = np.array([1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1], dtype=np.uint8)  # Баркер-13
MAX_POINTS = 4000         # сколько точек рисовать на графике амплитуды (остальное прореживается)
MAX_MARKS = 200           # сколько меток пакетов рисовать
MAX_SYMBOLS = 50000       # сколько символов рисовать на созвездии

# Режим live: сэмплы приходят от ./main rx по UDP (порты как в main.cpp)
UDP_PORTS = {"down": 5555, "up": 5556}
LIVE_SAMPLES = 200000     # длина кольцевого буфера, отсчетов (200 мс)

MODULATIONS = ("Авто", "BPSK", "QPSK")
# Идеальные точки созвездий при единичной средней мощности
IDEAL = {
    "BPSK": np.array([1, -1], dtype=complex),
    "QPSK": np.array([1 + 1j, -1 + 1j, -1 - 1j, 1 - 1j]) / np.sqrt(2),
}


# Генератор PRBS-9 как в main.cpp (x^9 + x^5 + 1, сброс в начале каждого пакета)
PRBS9_SEED = 0x1FF


def prbs9(n):
    lfsr = PRBS9_SEED
    bits = np.empty(n, dtype=np.uint8)
    for i in range(n):
        bit = ((lfsr >> 8) ^ (lfsr >> 4)) & 1
        lfsr = ((lfsr << 1) | bit) & 0x1FF
        bits[i] = bit
    return bits


def channel_path(channel):
    # Файл канала down/up: dev/build/rx_<канал>.pcm
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "build", f"rx_{channel}.pcm")


def load(name):
    # Формат файла: int16 little-endian I, Q, I, Q, ...
    data = np.fromfile(name, dtype='<i2').astype(np.float32)
    n = len(data) // 2
    return data[0:2 * n:2] + 1j * data[1:2 * n:2]


class Receiver:
    # Прием UDP-датаграмм в отдельном потоке и запись в кольцевой буфер.
    # GUI забирает копию последних LIVE_SAMPLES отсчетов через snapshot()
    def __init__(self, port, size=LIVE_SAMPLES):
        self.port = port
        self.buf = np.zeros(size, dtype=np.complex64)
        self.samples = 0          # всего принято отсчетов
        self.datagrams = 0
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Запас на случай, когда GUI надолго занял процессор
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        self.sock.bind(("127.0.0.1", port))
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while True:
            data = self.sock.recv(65536)
            # Формат как в файле: int16 little-endian I, Q, I, Q, ...
            raw = np.frombuffer(data, dtype='<i2')
            n = len(raw) // 2
            x = raw[0:2 * n:2] + 1j * raw[1:2 * n:2].astype(np.float32)
            size = len(self.buf)
            with self.lock:
                idx = (self.samples + np.arange(n)) % size
                self.buf[idx] = x
                self.samples += n
                self.datagrams += 1

    def snapshot(self):
        # Последние отсчеты по порядку, от старых к новым
        with self.lock:
            size = len(self.buf)
            if self.samples < size:
                return self.buf[:self.samples].copy()
            i = self.samples % size
            return np.concatenate((self.buf[i:], self.buf[:i]))


def find_packets(complex_signal, amplitude, sps, win, margin):
    # ---------------- Символьная синхронизация ----------------
    # Преамбула всегда BPSK (Баркер-13 в I), данные - BPSK или QPSK
    n_symbols = BUFFER // sps
    # Преамбула в отсчетах: 0 -> -1, 1 -> +1, каждый символ SPS раз
    template = np.repeat(2.0 * PREAMBLE - 1, sps)

    # 1. Поиск пакетов по энергии: порог посередине между шумом и сигналом
    env = np.convolve(amplitude, np.ones(win) / win, mode='same')
    lo, hi = np.percentile(env, [5, 95])
    above = env > (lo + hi) / 2
    rises = np.flatnonzero(above[1:] & ~above[:-1]) + 1

    starts = []
    taus = []
    symbols = []
    next_free = 0             # раньше этого отсчета новый пакет не ищем

    for r in rises:
        if r < next_free:     # фронт внутри уже найденного пакета
            continue
        if r < margin or r + BUFFER + margin > len(complex_signal):
            continue
        if above[r:r + BUFFER].mean() < 0.8:  # выброс шума
            continue
        seg = complex_signal[r - margin:r + BUFFER + margin]

        # 2. Корреляция с преамбулой: пик - начало пакета, фаза пика - фаза несущей
        corr = np.correlate(seg[:2 * margin + len(template)], template, mode='valid')
        p = int(np.argmax(np.abs(corr)))
        next_free = r - margin + p + BUFFER

        # 3. Согласованный фильтр: импульс прямоугольный, фильтр - сумма SPS отсчетов
        mf = np.convolve(seg, np.ones(sps), mode='full')

        # 4. Момент отсчета: сдвиг внутри символа с максимальной энергией.
        #    Окно сдвинуто на SPS // 2 из-за задержки фильтров
        start = p + sps // 2
        blk = mf[start:start + n_symbols * sps].reshape(n_symbols, sps)
        tau = int(np.argmax(np.sum(np.abs(blk) ** 2, axis=0)))

        # 5. Символы данных: поворот на фазу преамбулы, мощность приводится к 1
        data = blk[len(PREAMBLE):, tau] * np.exp(-1j * np.angle(corr[p]))
        data /= np.sqrt(np.mean(np.abs(data) ** 2))

        starts.append(r - margin + p)
        taus.append(sps // 2 + tau)
        symbols.append(data)

    return {
        "threshold": (lo + hi) / 2,
        "starts": np.array(starts, dtype=np.int64),
        "taus": taus,
        # строка - пакет, столбец - символ данных
        "symbols": np.array(symbols) if symbols else np.zeros((0, 0), dtype=complex),
    }


def detect_modulation(symbols):
    # Возведение в степень M: у BPSK |E[s^2]| ~ 1, у QPSK |E[s^2]| ~ 0, но |E[s^4]| ~ 1.
    # Считается по каждому пакету (фаза у пакетов разная), берется медиана
    m2 = np.median(np.abs(np.mean(symbols ** 2, axis=1)))
    m4 = np.median(np.abs(np.mean(symbols ** 4, axis=1)))
    if m2 > 0.5:
        return "BPSK", m2, m4
    if m4 > 0.5:
        return "QPSK", m2, m4
    return None, m2, m4


def evm(symbols, modulation):
    # EVM: среднеквадратичное отклонение от ближайшей идеальной точки, %
    ideal = IDEAL[modulation]
    nearest = ideal[np.argmin(np.abs(symbols[:, None] - ideal[None, :]), axis=1)]
    return 100 * np.sqrt(np.mean(np.abs(symbols - nearest) ** 2))


def decode(symbols, modulation):
    # Решение по знаку: BPSK - бит из I; QPSK - пара бит (I, Q), как в main.cpp.
    # symbols - строки-пакеты, результат - строки бит
    if modulation == "BPSK":
        return (symbols.real > 0).astype(np.uint8)
    bits = np.empty((symbols.shape[0], 2 * symbols.shape[1]), dtype=np.uint8)
    bits[:, 0::2] = symbols.real > 0
    bits[:, 1::2] = symbols.imag > 0
    return bits


def bit_errors(symbols, modulation):
    # Ошибки по пакетам: сравнение с эталонной PRBS-9
    bits = decode(symbols, modulation)
    return np.sum(bits != prbs9(bits.shape[1])[None, :], axis=1)


def decimate(y, i0, i1):
    # Отсчеты [i0, i1) для графика: если их больше MAX_POINTS,
    # каждую группу заменяем парой min/max, чтобы не терять пики
    seg = y[i0:i1]
    step = max(1, len(seg) // (MAX_POINTS // 2))
    if step == 1:
        x = np.arange(i0, i1, dtype=np.float64)
        return x * 1e3 / FS, seg.astype(np.float64)
    n = len(seg) // step
    blk = seg[:n * step].reshape(n, step)
    x = np.repeat(i0 + np.arange(n, dtype=np.float64) * step, 2)
    ys = np.empty(2 * n)
    ys[0::2] = blk.min(axis=1)
    ys[1::2] = blk.max(axis=1)
    return x * 1e3 / FS, ys


class App:
    def __init__(self, path, modulation="Авто", receiver=None):
        self.path = path
        self.receiver = receiver  # None - файл, иначе прием по UDP
        self.error = ""
        self.signal = None
        self.amplitude = None
        self.view = None          # (t0, t1) в мс, применяется на следующем кадре
        self.window = 20000       # ширина окна просмотра, отсчетов
        self.cur = (0.0, 0.0)     # видимый диапазон, мс

        # Параметры синхронизации
        self.sps = 10             # отсчетов на символ
        self.win = 64             # окно сглаживания огибающей
        self.margin = 100         # запас вокруг начала пакета
        self.result = None
        self.show_marks = True
        self.show_threshold = True

        # Созвездие
        self.mod_choice = MODULATIONS.index(modulation)
        self.detected = None      # (модуляция или None, m2, m4)
        self.only_visible = True  # только пакеты, видимые на графике амплитуды

        # Режим live
        self.paused = False
        self.period = 200         # период обновления, мс
        self.last_update = 0.0
        self.last_samples = 0
        self.rate = 0.0           # скорость приема, отсчетов/с
        self.rate_t = time.monotonic()
        self.rate_n = 0

        if receiver is None:
            self.load()

    def modulation(self):
        # Выбранная вручную или определенная автоматически модуляция
        if self.mod_choice > 0:
            return MODULATIONS[self.mod_choice]
        return self.detected[0] if self.detected else None

    def load(self):
        try:
            signal = load(self.path)
        except OSError as e:
            self.signal = None
            self.error = str(e)
            return
        if len(signal) == 0:
            self.signal = None
            self.error = "Файл пустой"
            return
        self.set_signal(signal)
        self.go_to_max()

    def set_signal(self, signal):
        self.error = ""
        self.signal = signal
        self.amplitude = np.abs(signal)
        self.find()

    def update_live(self):
        # Раз в self.period мс: копия кольцевого буфера и повторный поиск пакетов
        now = time.monotonic()
        if self.paused or now - self.last_update < self.period / 1e3:
            return
        if self.receiver.samples == self.last_samples:
            return
        self.last_update = now
        self.last_samples = self.receiver.samples
        first = self.signal is None
        self.set_signal(self.receiver.snapshot())
        if first:
            self.view = (0.0, len(self.signal) * 1e3 / FS)

    def find(self):
        self.result = find_packets(self.signal, self.amplitude,
                                   self.sps, self.win, self.margin)
        symbols = self.result["symbols"]
        self.detected = detect_modulation(symbols) if len(symbols) else None
        self.errors = {}          # модуляция -> ошибки по пакетам

    def packet_errors(self, mod):
        # Декодирование при первой надобности, отдельно для каждой модуляции
        if mod not in self.errors:
            self.errors[mod] = bit_errors(self.result["symbols"], mod)
        return self.errors[mod]

    def set_view(self, center):
        # Окно ширины self.window вокруг отсчета center
        start = max(0, center - self.window // 2)
        self.view = (start * 1e3 / FS, (start + self.window) * 1e3 / FS)

    def go_to_max(self):
        self.set_view(int(np.argmax(self.amplitude)))

    def file_settings(self):
        imgui.separator_text("Файл")
        for ch in ("down", "up"):
            if imgui.button(ch):
                self.path = channel_path(ch)
                self.load()
            imgui.same_line()
        imgui.new_line()
        imgui.set_next_item_width(-1)
        _, self.path = imgui.input_text("##path", self.path)
        if imgui.button("Загрузить"):
            self.load()

    def live_settings(self):
        r = self.receiver
        now = time.monotonic()
        if now - self.rate_t >= 1.0:
            self.rate = (r.samples - self.rate_n) / (now - self.rate_t)
            self.rate_t, self.rate_n = now, r.samples
        imgui.separator_text("Прием (live)")
        imgui.text(f"UDP 127.0.0.1:{r.port}")
        imgui.text(f"Принято: {r.samples / FS:.1f} с, датаграмм: {r.datagrams}")
        imgui.text(f"Скорость: {self.rate / 1e3:.0f} тыс. отсч./с")
        if self.rate == 0:
            imgui.text_colored(imgui.ImVec4(1, 0.8, 0.3, 1), "Нет данных: запущен ./main rx?")
        _, self.paused = imgui.checkbox("Пауза", self.paused)
        imgui.set_next_item_width(-1)
        _, self.period = imgui.slider_int("##period", self.period, 50, 2000,
                                          "обновление %d мс")

    def settings(self):
        if self.receiver is None:
            self.file_settings()
        else:
            self.live_settings()
        if self.error:
            imgui.text_colored(imgui.ImVec4(1, 0.4, 0.4, 1), self.error)
        if self.signal is None:
            return
        imgui.text(f"Отсчетов: {len(self.signal)} ({len(self.signal) / FS:.2f} с)")
        imgui.text(f"max = {self.amplitude.max():.0f}, "
                   f"среднее = {self.amplitude.mean():.1f}")

        imgui.separator_text("Вид")
        imgui.set_next_item_width(-1)
        _, self.window = imgui.slider_int("##window", self.window, 1000, 2000000,
                                          "окно %d отсч.", imgui.SliderFlags_.logarithmic)
        if imgui.button("К максимуму"):
            self.go_to_max()
        imgui.same_line()
        if imgui.button("Весь файл"):
            self.view = (0.0, len(self.signal) * 1e3 / FS)
        _, self.show_marks = imgui.checkbox("Начала пакетов", self.show_marks)
        _, self.show_threshold = imgui.checkbox("Порог детектора", self.show_threshold)

        imgui.separator_text("Синхронизация")
        imgui.set_next_item_width(120)
        _, self.sps = imgui.input_int("отсч. на символ", self.sps)
        imgui.set_next_item_width(120)
        _, self.win = imgui.input_int("окно огибающей", self.win)
        imgui.set_next_item_width(120)
        _, self.margin = imgui.input_int("запас", self.margin)
        self.sps = max(1, self.sps)
        self.win = max(1, self.win)
        self.margin = max(0, self.margin)
        if imgui.button("Найти пакеты"):
            self.find()

        imgui.separator_text("Созвездие")
        imgui.set_next_item_width(120)
        _, self.mod_choice = imgui.combo("модуляция", self.mod_choice, MODULATIONS)
        if self.detected:
            name, m2, m4 = self.detected
            imgui.text(f"Авто: {name or 'не определена'}")
            imgui.text(f"|E[s^2]| = {m2:.2f}, |E[s^4]| = {m4:.2f}")
        _, self.only_visible = imgui.checkbox("Только видимые пакеты", self.only_visible)

        if self.result is None:
            return
        imgui.separator_text("Результат")
        starts = self.result["starts"]
        if len(starts) == 0:
            imgui.text("Пакеты не найдены")
            return
        imgui.text(f"Найдено пакетов: {len(starts)}")
        imgui.text(f"Порог: {self.result['threshold']:.1f}")
        mod = self.modulation()
        if mod:
            symbols = self.result["symbols"]
            errors = self.packet_errors(mod)
            n_bits = symbols.shape[1] * (2 if mod == "QPSK" else 1)
            imgui.text(f"EVM ({mod}): {evm(symbols.ravel(), mod):.1f} %")
            imgui.text(f"Бит в пакете: {n_bits}")
            imgui.text(f"Ошибок: {errors.sum()} из {errors.size * n_bits}, "
                       f"BER = {errors.sum() / (errors.size * n_bits):.2e}")
            imgui.text(f"Пакетов без ошибок: {np.count_nonzero(errors == 0)} "
                       f"из {errors.size}")
        imgui.text("Момент отсчета в символе:")
        values, counts = np.unique(self.result["taus"], return_counts=True)
        for v, c in zip(values, counts):
            imgui.bullet_text(f"{v} - {c} пак.")
        if imgui.button("< Пред. пакет"):
            self.step_packet(-1)
        imgui.same_line()
        if imgui.button("След. пакет >"):
            self.step_packet(+1)

    def step_packet(self, direction):
        # Переход к соседнему пакету относительно центра текущего окна
        starts = self.result["starts"]
        center = (self.cur[0] + self.cur[1]) / 2 * FS / 1e3
        if direction > 0:
            i = np.searchsorted(starts, center + 1, side='left')
        else:
            i = np.searchsorted(starts, center - 1, side='left') - 1
        i = int(np.clip(i, 0, len(starts) - 1))
        self.set_view(int(starts[i]) + BUFFER // 2)

    def amplitude_plot(self, height):
        n = len(self.signal)
        if not implot.begin_plot("Амплитуда", imgui.ImVec2(-1, height)):
            return
        implot.setup_axes("Время, мс", "|I + jQ|", 0, implot.AxisFlags_.auto_fit)
        if self.view is not None:
            implot.setup_axis_limits(implot.ImAxis_.x1, *self.view, imgui.Cond_.always)
            self.view = None
        lim = implot.get_plot_limits().x
        self.cur = (lim.min, lim.max)
        # Рисуем только видимую часть файла
        i0 = int(np.clip(lim.min * FS / 1e3, 0, n))
        i1 = int(np.clip(lim.max * FS / 1e3 + 2, 0, n))
        if i1 > i0:
            implot.plot_line("|I + jQ|", *decimate(self.amplitude, i0, i1))
        if self.result is not None:
            if self.show_threshold:
                implot.plot_inf_lines("Порог", np.array([self.result["threshold"]]),
                                      implot.Spec(flags=implot.InfLinesFlags_.horizontal))
            starts = self.result["starts"]
            vis = starts[(starts >= i0) & (starts < i1)]
            if self.show_marks and 0 < len(vis) <= MAX_MARKS:
                implot.plot_inf_lines("Начало пакета", vis * 1e3 / FS)
        implot.end_plot()

    def constellation_plot(self):
        symbols = self.result["symbols"] if self.result else np.zeros((0, 0))
        if self.only_visible and len(symbols):
            # Пакеты, начало которых попало в окно графика амплитуды
            t = self.result["starts"] * 1e3 / FS
            symbols = symbols[(t >= self.cur[0]) & (t < self.cur[1])]
        symbols = symbols.ravel()
        step = max(1, len(symbols) // MAX_SYMBOLS)
        symbols = symbols[::step]

        if not implot.begin_plot(f"Созвездие ({len(symbols)} симв.)",
                                 imgui.ImVec2(-1, -1), implot.Flags_.equal):
            return
        implot.setup_axes("I", "Q")
        implot.setup_axes_limits(-2, 2, -2, 2, imgui.Cond_.once)
        # Оси - границы решений для BPSK (по I) и QPSK (по I и Q)
        grey = imgui.ImVec4(0.5, 0.5, 0.5, 1)
        implot.plot_inf_lines("##axes_x", np.array([0.0]), implot.Spec(line_color=grey))
        implot.plot_inf_lines("##axes_y", np.array([0.0]),
                              implot.Spec(line_color=grey,
                                          flags=implot.InfLinesFlags_.horizontal))
        if len(symbols):
            implot.plot_scatter("Символы", symbols.real.copy(), symbols.imag.copy(),
                                implot.Spec(marker=implot.Marker_.circle, marker_size=1.5,
                                            fill_alpha=0.3))
        mod = self.modulation()
        if mod:
            ideal = IDEAL[mod]
            implot.plot_scatter(f"Идеальные точки {mod}", ideal.real.copy(), ideal.imag.copy(),
                                implot.Spec(marker=implot.Marker_.cross, marker_size=10,
                                            line_weight=2))
        implot.end_plot()

    def plots(self):
        if self.signal is None:
            return
        self.amplitude_plot(imgui.get_content_region_avail().y * 0.4)
        self.constellation_plot()

    def gui(self):
        if self.receiver is not None:
            self.update_live()
        imgui.begin_child("settings", imgui.ImVec2(320, 0),
                          imgui.ChildFlags_.borders | imgui.ChildFlags_.resize_x)
        self.settings()
        imgui.end_child()
        imgui.same_line()
        imgui.begin_child("plots")
        self.plots()
        imgui.end_child()


def main():
    # Аргументы: [live] канал down/up (файл dev/build/rx_<канал>.pcm) или путь к файлу;
    # необязательно модуляция bpsk/qpsk, по умолчанию определяется автоматически.
    # live - прием от ./main rx по UDP вместо чтения файла
    args = sys.argv[1:]
    live = len(args) > 0 and args[0] == "live"
    if live:
        args = args[1:]
    arg = args[0] if args else "down"
    path = channel_path(arg) if arg in ("down", "up") else arg
    modulation = args[1].upper() if len(args) > 1 else "Авто"
    if modulation not in MODULATIONS:
        print(f"Неизвестная модуляция: {args[1]} (ожидается bpsk или qpsk)")
        return
    receiver = None
    title = "Приемник BPSK/QPSK"
    if live:
        if arg not in UDP_PORTS:
            print(f"Для live нужен канал down или up, а не {arg}")
            return
        try:
            receiver = Receiver(UDP_PORTS[arg])
        except OSError as e:
            print(f"Не удалось открыть UDP-порт {UDP_PORTS[arg]}: {e}")
            return
        title += f" (live, {arg})"
    app = App(path, modulation, receiver)
    immapp.run(gui_function=app.gui, window_title=title,
               window_size=(1400, 850), with_implot=True)


if __name__ == "__main__":
    main()
