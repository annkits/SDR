import os
import socket
import sys
import threading
import time

import numpy as np
from imgui_bundle import hello_imgui, imgui, immapp, implot
from imgui_bundle import icons_fontawesome_4 as fa  # иконки из шрифта hello_imgui по умолчанию

FS = 1e6                  # частота дискретизации, как в main.cpp
BUFFER = 1920             # отсчетов в пакете
PREAMBLE = np.array([1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1], dtype=np.uint8)  # Баркер-13
MAX_POINTS = 4000         # сколько точек рисовать на графике амплитуды (остальное прореживается)
MAX_MARKS = 200           # сколько меток пакетов рисовать
MAX_SYMBOLS = 50000       # сколько символов рисовать на созвездии

# Режим live: сэмплы приходят от ./main rx по UDP (порты как в main.cpp)
UDP_PORTS = {"down": 5555, "up": 5556}
UDP_HEADER = 8            # uint64 в начале датаграммы: номер ее первого отсчета
LIVE_SAMPLES = 500000     # длина кольцевого буфера, отсчетов (500 мс)
LIVE_LAG = 100000         # наименьшее отставание правого края графика от приема, отсчетов
                          # (100 мс): запас на то, что отсчеты приходят пачками
LIVE_PERIOD = 0.05        # период поиска пакетов в новых отсчетах, с
# Управление ./main по UDP: порт данных канала + 10 для rx, + 20 для tx (как в main.cpp)
CTRL_OFFSET = {"rx": 10, "tx": 20}
CTRL_POLL = 0.5           # период запроса настроек, с
CTRL_SEND = 0.05          # изменения отправляются не чаще, с
CTRL_HOLD = 1.0           # сколько после изменения показывать свое значение, а не ответ процесса

MODULATIONS = ("Авто", "BPSK", "QPSK")
# Идеальные точки созвездий при единичной средней мощности
IDEAL = {
    "BPSK": np.array([1, -1], dtype=complex),
    "QPSK": np.array([1 + 1j, -1 + 1j, -1 - 1j, 1 - 1j]) / np.sqrt(2),
}

# Цвета интерфейса и графиков
ACCENT = imgui.ImVec4(0.36, 0.62, 1.00, 1)
GOOD = imgui.ImVec4(0.42, 0.82, 0.48, 1)
WARN = imgui.ImVec4(0.98, 0.76, 0.30, 1)
BAD = imgui.ImVec4(0.96, 0.42, 0.42, 1)
MARK = imgui.ImVec4(0.42, 0.82, 0.48, 0.65)
# в старых imgui-bundle флаг рамки называется border, в новых — borders
CHILD_BORDERS = getattr(imgui.ChildFlags_, "borders", None) or imgui.ChildFlags_.border
# imgui до 1.92 загружает из шрифта только латиницу, кириллица выводится как "?"
OLD_FONTS = tuple(int(x) for x in imgui.get_version().split(".")[:2]) < (1, 92)
IDEAL_COLOR = imgui.ImVec4(1.00, 0.55, 0.20, 1)
GREY = imgui.ImVec4(0.5, 0.5, 0.5, 1)


def style(flags=0, **kw):
    """Стиль элемента графика: implot.Spec в новых imgui-bundle,
    set_next_*_style + flags в старых (до implot 0.17).
    Возвращает аргументы для **, вызывать прямо в вызове plot_*."""
    if hasattr(implot, "Spec"):
        return {"spec": implot.Spec(flags=flags, **kw)}
    line = {}
    if "line_color" in kw:
        line["col"] = kw["line_color"]
    if "line_weight" in kw:
        line["weight"] = kw["line_weight"]
    if line:
        implot.set_next_line_style(**line)
    fill = {}
    if "fill_color" in kw:
        fill["col"] = kw["fill_color"]
    if "fill_alpha" in kw:
        fill["alpha_mod"] = kw["fill_alpha"]
    if fill:
        implot.set_next_fill_style(**fill)
    if "marker" in kw:
        marker = {"marker": kw["marker"]}
        if "marker_size" in kw:
            marker["size"] = kw["marker_size"]
        if "marker_fill_color" in kw:
            marker["fill"] = kw["marker_fill_color"]
        if "marker_line_color" in kw:
            marker["outline"] = kw["marker_line_color"]
        if "line_weight" in kw:
            marker["weight"] = kw["line_weight"]
        implot.set_next_marker_style(**marker)
    return {"flags": flags}


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
    # Отсчет с номером k (с начала приема в ./main rx) лежит в buf[k % size],
    # GUI забирает копию последних LIVE_SAMPLES отсчетов через snapshot()
    def __init__(self, port, size=LIVE_SAMPLES):
        self.port = port
        self.buf = np.zeros(size, dtype=np.complex64)
        self.samples = 0          # номер отсчета, следующего за последним принятым
        self.received = 0         # всего принято отсчетов
        self.lost = 0             # пропущено отсчетов (потерянные датаграммы)
        self.datagrams = 0
        self.restarts = 0         # сколько раз ./main rx перезапускался
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Запас на случай, когда GUI надолго занял процессор
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        self.sock.bind(("127.0.0.1", port))
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while True:
            data = self.sock.recv(65536)
            if len(data) < UDP_HEADER:
                continue
            pos = int(np.frombuffer(data, dtype='<u8', count=1)[0])
            # Дальше как в файле: int16 little-endian I, Q, I, Q, ...
            raw = np.frombuffer(data, dtype='<i2', offset=UDP_HEADER)
            n = len(raw) // 2
            x = raw[0:2 * n:2] + 1j * raw[1:2 * n:2].astype(np.float32)
            size = len(self.buf)
            with self.lock:
                if pos + n <= self.samples - size:
                    # Номер меньше, чем помещается в буфер: ./main rx запущен заново
                    self.buf[:] = 0
                    self.samples = 0
                    self.restarts += 1
                if pos > self.samples:
                    # Датаграммы потерялись: на их месте нули, а не старые отсчеты
                    gap = pos - self.samples
                    self.buf[(self.samples + np.arange(min(gap, size))) % size] = 0
                    self.lost += gap
                self.buf[(pos + np.arange(n)) % size] = x
                self.samples = max(self.samples, pos + n)
                self.received += n
                self.datagrams += 1

    def snapshot(self, start=0):
        # (номер первого отсчета, отсчеты от start до последнего принятого по порядку).
        # Если start уже вышел из буфера - с самого старого отсчета в буфере
        with self.lock:
            size = len(self.buf)
            first = max(start, self.samples - size, 0)
            n = self.samples - first
            i = first % size
            if i + n <= size:
                return first, self.buf[i:i + n].copy()
            return first, np.concatenate((self.buf[i:], self.buf[:i + n - size]))


class Control:
    # Настройка ./main rx или ./main tx во время работы (см. main.cpp): команда
    # "ключ=значение ..." на управляющий порт, в ответ - текущие настройки в том же виде.
    # Изменения копятся и уходят не чаще CTRL_SEND, без изменений раз в CTRL_POLL
    # уходит пустая команда - запрос настроек
    def __init__(self, port):
        self.port = port
        self.state = {}           # последние настройки от процесса, строки
        self.replied = -1e9       # когда пришел последний ответ
        self.wanted = {}          # изменения, еще не отправленные
        self.shown = {}           # значения в виджетах
        self.edited = {}          # ключ -> когда пользователь менял его последний раз
        self.sent = -1e9
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while True:
            try:
                data = self.sock.recv(4096)
            except OSError:
                continue
            pairs = (p.split("=", 1) for p in data.decode(errors="replace").split())
            self.state = {p[0]: p[1] for p in pairs if len(p) == 2}
            self.replied = time.monotonic()

    def connected(self):
        return time.monotonic() - self.replied < 3 * CTRL_POLL

    def get(self, key, kind=float):
        # Значение для виджета: свое, пока пользователь его недавно менял, иначе от процесса
        if time.monotonic() - self.edited.get(key, -1e9) > CTRL_HOLD and key in self.state:
            try:
                self.shown[key] = kind(float(self.state[key])) if kind is not str else self.state[key]
            except ValueError:
                pass
        return self.shown.get(key, kind())

    def set(self, key, value):
        self.shown[key] = value
        self.wanted[key] = value
        self.edited[key] = time.monotonic()

    def update(self):
        # Каждый кадр: отправка накопленных изменений или запрос настроек
        now = time.monotonic()
        if (self.wanted and now - self.sent >= CTRL_SEND) or now - self.sent >= CTRL_POLL:
            cmd = " ".join(f"{k}={v}" for k, v in self.wanted.items())
            self.wanted = {}
            self.sent = now
            try:
                self.sock.sendto(cmd.encode(), ("127.0.0.1", self.port))
            except OSError:
                pass


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
    # Доля отсчетов выше порога на любом отрезке - через накопленную сумму
    above_sum = np.concatenate(([0], np.cumsum(above)))

    starts = []
    taus = []
    symbols = []
    next_free = 0             # раньше этого отсчета новый пакет не ищем

    for r in rises:
        if r < next_free:     # фронт внутри уже найденного пакета
            continue
        if r < margin or r + BUFFER + margin > len(complex_signal):
            continue
        if above_sum[r + BUFFER] - above_sum[r] < 0.8 * BUFFER:  # выброс шума
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


def merge_packets(old, new, oldest):
    # Live: пакеты, найденные раньше (кроме начавшихся до отсчета oldest - они ушли
    # из буфера), плюс новые. Участки поиска перекрываются, поэтому новый пакет,
    # начавшийся не позже последнего старого (с точностью до полпакета), - повтор
    if old is None:
        return new
    keep = old["starts"] >= oldest
    last = old["starts"][-1] if len(old["starts"]) else -BUFFER
    add = new["starts"] > last + BUFFER // 2
    symbols = [s for s in (old["symbols"][keep], new["symbols"][add]) if s.size]
    return {
        "threshold": new["threshold"],
        "starts": np.concatenate((old["starts"][keep], new["starts"][add])),
        "taus": [t for t, k in zip(old["taus"], keep) if k]
                + [t for t, a in zip(new["taus"], add) if a],
        "symbols": np.concatenate(symbols) if symbols else np.zeros((0, 0), dtype=complex),
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


def decimate(y, i0, i1, offset=0):
    # Отсчеты [i0, i1) для графика: если их больше MAX_POINTS,
    # каждую группу заменяем парой min/max, чтобы не терять пики.
    # offset - номер отсчета y[0], время по оси считается от него
    seg = y[i0:i1]
    step = max(1, len(seg) // (MAX_POINTS // 2))
    if step == 1:
        x = offset + np.arange(i0, i1, dtype=np.float64)
        return x * 1e3 / FS, seg.astype(np.float64)
    n = len(seg) // step
    blk = seg[:n * step].reshape(n, step)
    x = np.repeat(offset + i0 + np.arange(n, dtype=np.float64) * step, 2)
    ys = np.empty(2 * n)
    ys[0::2] = blk.min(axis=1)
    ys[1::2] = blk.max(axis=1)
    return x * 1e3 / FS, ys


# ---------------- Оформление ----------------

def setup_style():
    # Тема и отступы; вызывается на первом кадре, когда контекст ImGui уже создан
    hello_imgui.apply_theme(hello_imgui.ImGuiTheme_.darcula_darker)
    s = imgui.get_style()
    s.window_padding = imgui.ImVec2(10, 10)
    s.frame_padding = imgui.ImVec2(8, 4)
    s.item_spacing = imgui.ImVec2(8, 6)
    s.cell_padding = imgui.ImVec2(4, 3)
    s.frame_rounding = 4
    s.child_rounding = 6
    s.grab_rounding = 4
    s.popup_rounding = 4
    s.scrollbar_rounding = 6
    s.window_rounding = 6
    implot.get_style().plot_padding = imgui.ImVec2(8, 8)
    implot.get_style().fit_padding = imgui.ImVec2(0, 0.08)  # запас над столбцами при подгонке


def tooltip(text):
    # Подсказка при наведении на предыдущий элемент, с переносом строк
    if text and imgui.begin_item_tooltip():
        imgui.push_text_wrap_pos(imgui.get_font_size() * 22)
        imgui.text_unformatted(text)
        imgui.pop_text_wrap_pos()
        imgui.end_tooltip()


def section(icon, title):
    # Сворачиваемый раздел панели, по умолчанию раскрыт
    return imgui.collapsing_header(f"{icon}  {title}", imgui.TreeNodeFlags_.default_open)


def begin_props(name):
    # Таблица "подпись - значение" внутри раздела
    if not imgui.begin_table(name, 2):
        return False
    imgui.table_setup_column("label", imgui.TableColumnFlags_.width_stretch, 0.48)
    imgui.table_setup_column("value", imgui.TableColumnFlags_.width_stretch, 0.52)
    return True


def prop(label, help=None):
    # Строка таблицы: подпись (со значком подсказки) слева, следующий виджет справа
    imgui.table_next_row()
    imgui.table_next_column()
    imgui.align_text_to_frame_padding()
    imgui.text(label)
    tooltip(help)
    if help:
        imgui.same_line(0, 4)
        imgui.text_disabled(fa.ICON_FA_QUESTION_CIRCLE)
        tooltip(help)
    imgui.table_next_column()
    imgui.set_next_item_width(-1)


def value(label, text, color=None, help=None):
    prop(label, help)
    if color is None:
        imgui.text(text)
    else:
        imgui.text_colored(color, text)


def half_width():
    # Ширина кнопки, чтобы две поместились в строку
    return (imgui.get_content_region_avail().x - imgui.get_style().item_spacing.x) / 2


def wide_button(label, width=-1.0):
    return imgui.button(label, imgui.ImVec2(width, 0))


def grade(x, good, warn):
    # Цвет показателя: меньше good - хорошо, меньше warn - так себе, иначе плохо
    return GOOD if x < good else WARN if x < warn else BAD


class App:
    def __init__(self, path, modulation="Авто", receiver=None):
        self.path = path
        self.receiver = receiver  # None - файл, иначе прием по UDP
        self.error = ""
        self.signal = None
        self.amplitude = None
        self.offset = 0           # номер отсчета signal[0] (в live - с начала приема)
        self.view = None          # (t0, t1) в мс, применяется на следующем кадре
        self.window = 20000       # ширина окна просмотра, отсчетов
        self.cur = (0.0, 0.0)     # видимый диапазон, мс
        self.styled = False       # тема применяется на первом кадре

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
        self.force_find = False   # искать заново во всем буфере (изменились параметры)
        self.covered = None       # до этого отсчета поиск пакетов уже прошел
        self.lag = float(LIVE_LAG)  # отставание правого края графика от приема, отсчетов
        self.ymax = 0.0           # предел оси амплитуды, чтобы она не прыгала
        self.play = 0.0           # правый край графика, номер отсчета
        self.restarts = 0
        self.last_samples = 0
        self.last_data = 0.0      # когда последний раз пришли новые отсчеты
        self.rate = 0.0           # скорость приема, отсчетов/с
        self.rate_t = time.monotonic()
        self.rate_n = 0
        self.pending = None       # результат поиска из фонового потока
        self.pending_lock = threading.Lock()
        # Настройка ./main rx и ./main tx этого канала во время работы
        self.controls = {}

        if receiver is None:
            self.load()
        else:
            self.controls = {role: Control(receiver.port + off) for role, off in CTRL_OFFSET.items()}
            threading.Thread(target=self.live_worker, daemon=True).start()

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

    def live_worker(self):
        # Фоновый поток: раз в LIVE_PERIOD ищет пакеты только в новых отсчетах
        # кольцевого буфера и добавляет их к найденным раньше. Так поиск дешевый
        # и результат успевает за графиком, а отрисовка кадров не задерживается
        r = self.receiver
        found = None              # накопленный результат
        params = None             # параметры, с которыми он получен
        searched = 0              # до этого отсчета буфер уже просмотрен
        restarts = r.restarts
        while True:
            time.sleep(LIVE_PERIOD)
            sps, win, margin = self.sps, self.win, self.margin
            if self.force_find or r.restarts != restarts or (sps, win, margin) != params:
                # Параметры изменились или ./main rx перезапущен: заново по всему буферу
                self.force_find = False
                restarts = r.restarts
                found, params, searched = None, (sps, win, margin), 0
            elif self.paused or r.samples == searched:
                continue
            # Захватываем и конец прошлого участка: пакет, обрезанный его краем, найдется целиком
            offset, signal = r.snapshot(searched - BUFFER - 2 * margin)
            if len(signal) < BUFFER + 2 * margin:
                continue
            end = offset + len(signal)
            result = find_packets(signal, np.abs(signal), sps, win, margin)
            result["starts"] += offset
            found = merge_packets(found, result, end - LIVE_SAMPLES)
            searched = end
            symbols = found["symbols"]
            detected = detect_modulation(symbols) if len(symbols) else None
            # Пакет, начавшийся позже end - BUFFER - margin, еще не может быть найден
            with self.pending_lock:
                self.pending = (found, detected, end - BUFFER - margin)

    def update_live(self):
        # Каждый кадр: свежие отсчеты из кольцевого буфера и плавный сдвиг окна
        r = self.receiver
        now = time.monotonic()
        dt = imgui.get_io().delta_time
        if r.restarts != self.restarts:
            # ./main rx перезапущен, номера отсчетов начались с нуля
            self.restarts = r.restarts
            self.play = 0.0
            self.result = None
            self.detected = None
            self.covered = None
        with self.pending_lock:
            pending, self.pending = self.pending, None
        if pending is not None and not self.paused:
            self.result, self.detected, self.covered = pending
            self.errors = {}
            if self.amplitude is not None:
                self.ymax = 1.1 * float(self.amplitude.max())
        if self.paused:
            return
        if r.samples != self.last_samples:
            self.last_samples = r.samples
            self.last_data = now
            self.offset, self.signal = r.snapshot()
            self.amplitude = np.abs(self.signal)
        if self.signal is None:
            return

        # Отставание графика: не меньше LIVE_LAG и такое, чтобы правый край не заходил
        # туда, где пакеты еще не искали. Растет сразу, уменьшается медленно
        lag = LIVE_LAG
        if self.covered is not None:
            lag = max(lag, r.samples - self.covered + 0.02 * FS)
        self.lag = min(max(lag, self.lag - 0.1 * FS * dt), LIVE_SAMPLES / 2)

        # Правый край графика идет равномерно со скоростью FS и плавно
        # подтягивается к принятому минус отставание: отсчеты приходят пачками,
        # и если привязать край прямо к ним, график будет дергаться
        target = r.samples - self.lag
        if now - self.last_data < 0.15:
            self.play += dt * FS
        err = target - self.play
        if abs(err) > 2 * LIVE_LAG:
            self.play = target    # сильно отстали или убежали - прыжок
        else:
            self.play += err * min(1.0, 2 * dt)
        self.window = min(self.window, int(LIVE_SAMPLES - self.lag))
        self.view = ((self.play - self.window) * 1e3 / FS, self.play * 1e3 / FS)

    def find(self):
        if self.receiver is not None and not self.paused:
            self.force_find = True    # live: поиск в фоновом потоке
            return
        self.result = find_packets(self.signal, self.amplitude,
                                   self.sps, self.win, self.margin)
        self.result["starts"] += self.offset
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
        start = max(self.offset, center - self.window // 2)
        self.view = (start * 1e3 / FS, (start + self.window) * 1e3 / FS)

    def go_to_max(self):
        self.set_view(self.offset + int(np.argmax(self.amplitude)))

    def file_settings(self):
        if not section(fa.ICON_FA_FOLDER_OPEN, "Файл"):
            return
        w = half_width()
        for i, ch in enumerate(("down", "up")):
            if i:
                imgui.same_line()
            current = os.path.abspath(self.path) == channel_path(ch)
            if current:
                imgui.push_style_color(imgui.Col_.button, imgui.get_style_color_vec4(imgui.Col_.button_active))
            if wide_button(f"Канал {ch}", w):
                self.path = channel_path(ch)
                self.load()
            if current:
                imgui.pop_style_color()
            tooltip(channel_path(ch))
        imgui.set_next_item_width(-1)
        _, self.path = imgui.input_text_with_hint("##path", "путь к .pcm", self.path)
        enter = imgui.is_item_focused() and imgui.is_key_pressed(imgui.Key.enter)
        tooltip("Файл int16 I, Q, I, Q, ... Enter - загрузить")
        if wide_button(f"{fa.ICON_FA_DOWNLOAD}  Загрузить") or enter:
            self.load()

    def live_settings(self):
        r = self.receiver
        now = time.monotonic()
        if now - self.rate_t >= 1.0:
            self.rate = (r.received - self.rate_n) / (now - self.rate_t)
            self.rate_t, self.rate_n = now, r.received
        if not section(fa.ICON_FA_RSS, "Прием (live)"):
            return
        if self.paused:
            imgui.text_colored(WARN, f"{fa.ICON_FA_PAUSE}  Пауза")
        elif self.rate == 0:
            imgui.text_colored(BAD, f"{fa.ICON_FA_CIRCLE}  Нет данных")
            tooltip("Отсчеты не приходят. Запущен ли ./main rx?")
        else:
            imgui.text_colored(GOOD, f"{fa.ICON_FA_CIRCLE}  Идет прием")
        if wide_button(f"{fa.ICON_FA_PLAY}  Продолжить" if self.paused
                       else f"{fa.ICON_FA_PAUSE}  Пауза"):
            self.paused = not self.paused
        tooltip("Пробел. На паузе график можно двигать и масштабировать")
        if begin_props("live"):
            value("Порт", f"UDP {r.port}")
            value("Принято", f"{r.received / FS:.1f} с")
            value("Датаграмм", f"{r.datagrams}")
            value("Скорость", f"{self.rate / 1e3:.0f} тыс. отсч./с",
                  GOOD if self.rate > 0.95 * FS else WARN)
            lost = 100 * r.lost / max(1, r.samples)
            value("Потеряно", f"{r.lost} ({lost:.2f} %)", GOOD if r.lost == 0 else grade(lost, 1, 5),
                  "Отсчеты из потерянных датаграмм UDP, на их месте нули")
            value("Задержка графика", f"{self.lag * 1e3 / FS:.0f} мс",
                  help="Насколько правый край графика отстает от приема: запас на "
                  "неравномерный приход отсчетов и на поиск пакетов в новых отсчетах")
            imgui.end_table()

    def sdr_settings(self, role):
        # Настройки ./main rx или ./main tx этого канала, применяются во время работы
        c = self.controls[role]
        c.update()
        rx = role == "rx"
        if not section(fa.ICON_FA_SIGNAL if rx else fa.ICON_FA_WIFI,
                       "Приемник SDR" if rx else "Передатчик SDR"):
            return
        if not c.connected():
            channel = next(k for k, v in UDP_PORTS.items() if v == self.receiver.port)
            imgui.text_disabled(f"{fa.ICON_FA_CIRCLE}  Нет связи с ./main {role}")
            tooltip(f"Нет ответа на управляющий порт UDP {c.port}. "
                    f"Запустите ./main {role} <uri> {channel} (сборка с управлением)")
            return
        imgui.text_colored(GOOD, f"{fa.ICON_FA_CIRCLE}  ./main {role} на связи")
        tooltip("Изменения применяются сразу, без перезапуска")
        if not begin_props(role):
            return
        prop("Частота", "Частота несущей. Ввод - Enter; +/- меняют на 100 Гц, с Ctrl - на 10 кГц"
             + (". Подстройкой частоты приемника можно убрать сдвиг частоты между "
                "SDR, из-за которого созвездие - кольцо" if rx else ""))
        changed, f = imgui.input_double(f"##freq_{role}", c.get("freq"), 100, 10000, "%.0f Гц",
                                        imgui.InputTextFlags_.enter_returns_true)
        if changed:
            c.set("freq", float(round(f)))
        agc = False
        if rx:
            prop("АРУ", "Автоматическая регулировка усиления AD9361. "
                 "Пока включена, усиление приемник выбирает сам")
            changed, agc = imgui.checkbox("##agc", bool(c.get("agc", int)))
            if changed:
                c.set("agc", int(agc))
        prop("Усиление", "Усиление приемного тракта" if rx else
             "Драйвер переводит в ослабление передатчика: максимум - без ослабления")
        imgui.begin_disabled(agc)
        changed, g = imgui.slider_float(f"##gain_{role}", c.get("gain"),
                                        c.get("gain_min"), c.get("gain_max"), "%.0f дБ")
        if changed:
            c.set("gain", float(round(g)))
        imgui.end_disabled()
        if not rx:
            amp_max = c.get("amp_max", int)
            prop("Амплитуда", f"Уровень символа в отсчетах 12-битного ЦАП (до {amp_max}). "
                 "У QPSK по I и Q - амплитуда / sqrt(2), мощность та же")
            changed, a = imgui.slider_int("##amp", c.get("amp", int), 0, amp_max)
            if changed:
                c.set("amp", a)
            prop("Модуляция")
            mods = ("bpsk", "qpsk")
            mod = c.get("mod", str)
            changed, i = imgui.combo("##tx_mod", mods.index(mod) if mod in mods else 0,
                                     ("BPSK", "QPSK"))
            if changed:
                c.set("mod", mods[i])
        imgui.end_table()

    def view_settings(self):
        if not section(fa.ICON_FA_EYE, "Вид"):
            return
        if begin_props("view"):
            n = len(self.signal)
            value("Длительность", f"{n / FS:.2f} с ({n} отсч.)")
            value("Амплитуда", f"макс. {self.amplitude.max():.0f}, ср. {self.amplitude.mean():.1f}")
            max_window = 2000000 if self.receiver is None else int(LIVE_SAMPLES - self.lag)
            prop("Ширина окна", "Сколько отсчетов видно на графике амплитуды")
            _, self.window = imgui.slider_int("##window", self.window, 1000, max_window,
                                              "%d отсч.", imgui.SliderFlags_.logarithmic)
            imgui.end_table()
        w = half_width()
        if wide_button(f"{fa.ICON_FA_CROSSHAIRS}  К максимуму", w):
            self.go_to_max()
        imgui.same_line()
        if wide_button(f"{fa.ICON_FA_EXPAND}  " + ("Весь файл" if self.receiver is None
                                                   else "Весь буфер"), w):
            self.view = (self.offset * 1e3 / FS, (self.offset + len(self.signal)) * 1e3 / FS)
        _, self.show_marks = imgui.checkbox("Начала пакетов", self.show_marks)
        imgui.same_line()
        _, self.show_threshold = imgui.checkbox("Порог", self.show_threshold)
        tooltip("Порог детектора энергии: посередине между шумом и сигналом")

    def sync_settings(self):
        if not section(fa.ICON_FA_SLIDERS_H, "Поиск пакетов"):
            return
        if begin_props("sync"):
            prop("Отсч. на символ", "SPS: должно совпадать с передатчиком (main.cpp)")
            _, self.sps = imgui.input_int("##sps", self.sps)
            prop("Окно огибающей", "Длина скользящего среднего амплитуды, отсчетов. "
                 "Сглаживает огибающую перед сравнением с порогом")
            _, self.win = imgui.input_int("##win", self.win)
            prop("Запас", "Сколько отсчетов до и после фронта пакета "
                 "просматривать при поиске преамбулы")
            _, self.margin = imgui.input_int("##margin", self.margin)
            imgui.end_table()
        self.sps = max(1, self.sps)
        self.win = max(1, self.win)
        self.margin = max(0, self.margin)
        if wide_button(f"{fa.ICON_FA_SEARCH}  Найти пакеты"):
            self.find()
        if self.receiver is not None:
            tooltip("В live поиск идет сам; кнопка запускает его заново по всему буферу")

    def modulation_settings(self):
        if not section(fa.ICON_FA_BULLSEYE, "Созвездие"):
            return
        if begin_props("modulation"):
            prop("Модуляция")
            _, self.mod_choice = imgui.combo("##modulation", self.mod_choice, MODULATIONS)
            if self.detected:
                name, m2, m4 = self.detected
                value("Определена", name or "нет", ACCENT if name else WARN,
                      f"|E[s^2]| = {m2:.2f}, |E[s^4]| = {m4:.2f}\n"
                      "BPSK: |E[s^2]| ~ 1. QPSK: |E[s^2]| ~ 0, |E[s^4]| ~ 1. "
                      "Если созвездие вращается (кольцо), оба близки к 0")
            imgui.end_table()
        _, self.only_visible = imgui.checkbox("Только видимые пакеты", self.only_visible)
        tooltip("Рисовать символы только пакетов, видимых на графике амплитуды")

    def results(self):
        if self.result is None or not section(fa.ICON_FA_CHECK_CIRCLE, "Результат"):
            return
        starts = self.result["starts"]
        if len(starts) == 0:
            imgui.text_colored(WARN, "Пакеты не найдены")
            return
        if begin_props("result"):
            value("Найдено пакетов", f"{len(starts)}")
            value("Порог", f"{self.result['threshold']:.1f}")
            mod = self.modulation()
            if mod:
                symbols = self.result["symbols"]
                errors = self.packet_errors(mod)
                n_bits = symbols.shape[1] * (2 if mod == "QPSK" else 1)
                total = errors.size * n_bits
                ber = errors.sum() / total
                e = evm(symbols.ravel(), mod)
                value("EVM", f"{e:.1f} %", grade(e, 15, 35),
                      "Среднеквадратичное отклонение символов от ближайшей идеальной точки")
                value("BER", f"{ber:.2e}", grade(ber, 1e-4, 1e-2),
                      f"Ошибок {errors.sum()} из {total} бит (эталон - PRBS-9)")
                ok = np.count_nonzero(errors == 0)
                value("Без ошибок", f"{ok} из {errors.size} ({100 * ok / errors.size:.0f} %)",
                      grade(1 - ok / errors.size, 0.05, 0.5))
                value("Бит в пакете", f"{n_bits}")
            imgui.end_table()
        w = half_width()
        if wide_button(f"{fa.ICON_FA_CHEVRON_LEFT}  Пред. пакет", w):
            self.step_packet(-1)
        tooltip("Стрелка влево")
        imgui.same_line()
        if wide_button(f"След. пакет  {fa.ICON_FA_CHEVRON_RIGHT}", w):
            self.step_packet(+1)
        tooltip("Стрелка вправо")

    def hints(self):
        if not imgui.collapsing_header(f"{fa.ICON_FA_KEYBOARD}  Управление"):
            return
        for line in ("Колесо мыши - масштаб",
                     "ЛКМ + перетаскивание - сдвиг",
                     "ПКМ + перетаскивание - увеличить область",
                     "Двойной клик - вписать все",
                     "Стрелки влево/вправо - соседний пакет",
                     "Пробел - пауза (live)"):
            imgui.bullet_text(line)

    def settings(self):
        if self.receiver is None:
            self.file_settings()
        else:
            self.live_settings()
            self.sdr_settings("rx")
            self.sdr_settings("tx")
        if self.error:
            imgui.text_colored(BAD, f"{fa.ICON_FA_EXCLAMATION_TRIANGLE}  {self.error}")
        if self.signal is not None:
            self.view_settings()
            self.sync_settings()
            self.modulation_settings()
            self.results()
        self.hints()

    def hotkeys(self):
        if imgui.get_io().want_text_input:
            return
        if self.result is not None and len(self.result["starts"]):
            if imgui.is_key_pressed(imgui.Key.left_arrow):
                self.step_packet(-1)
            if imgui.is_key_pressed(imgui.Key.right_arrow):
                self.step_packet(+1)
        if self.receiver is not None and imgui.is_key_pressed(imgui.Key.space, False):
            self.paused = not self.paused

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
        live = self.receiver is not None
        # В live ось амплитуды фиксирована по всему буферу: подгонка по видимой
        # части меняла бы масштаб в каждом кадре
        implot.setup_axes("Время, мс", "|I + jQ|", 0,
                          0 if live else implot.AxisFlags_.auto_fit)
        implot.setup_legend(implot.Location_.north_east, implot.LegendFlags_.horizontal)
        if live and not self.paused and self.ymax > 0:
            implot.setup_axis_limits(implot.ImAxis_.y1, 0, self.ymax, imgui.Cond_.always)
        if self.view is not None:
            implot.setup_axis_limits(implot.ImAxis_.x1, *self.view, imgui.Cond_.always)
            self.view = None
        lim = implot.get_plot_limits().x
        self.cur = (lim.min, lim.max)
        # Рисуем только видимую часть сигнала; номера отсчетов считаются от self.offset
        i0 = int(np.clip(lim.min * FS / 1e3 - self.offset, 0, n))
        i1 = int(np.clip(lim.max * FS / 1e3 + 2 - self.offset, 0, n))
        if i1 > i0:
            implot.plot_line("|I + jQ|", *decimate(self.amplitude, i0, i1, self.offset),
                             **style(line_color=ACCENT))
        if self.result is not None:
            if self.show_threshold:
                implot.plot_inf_lines("Порог", np.array([self.result["threshold"]]),
                                      **style(line_color=BAD, line_weight=1.5,
                                              flags=implot.InfLinesFlags_.horizontal))
            starts = self.result["starts"]
            vis = starts[(starts >= self.offset + i0) & (starts < self.offset + i1)]
            if self.show_marks and 0 < len(vis) <= MAX_MARKS:
                implot.plot_inf_lines("Начало пакета", vis * 1e3 / FS,
                                      **style(line_color=MARK, line_weight=1.5))
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

        # После ### - постоянный ID: иначе при каждом изменении числа символов
        # ImPlot создает новый график и сбрасывает масштаб
        if not implot.begin_plot(f"Созвездие ({len(symbols)} симв.)###constellation",
                                 imgui.ImVec2(-1, -1), implot.Flags_.equal):
            return
        implot.setup_axes("I", "Q")
        implot.setup_axes_limits(-1.6, 1.6, -1.6, 1.6, imgui.Cond_.once)
        implot.setup_legend(implot.Location_.north_east)
        # Оси - границы решений для BPSK (по I) и QPSK (по I и Q)
        implot.plot_inf_lines("##axes_x", np.array([0.0]), **style(line_color=GREY))
        implot.plot_inf_lines("##axes_y", np.array([0.0]),
                              **style(line_color=GREY,
                                      flags=implot.InfLinesFlags_.horizontal))
        if len(symbols):
            implot.plot_scatter("Символы", symbols.real.copy(), symbols.imag.copy(),
                                **style(marker=implot.Marker_.circle, marker_size=1.5,
                                        marker_fill_color=ACCENT, marker_line_color=ACCENT,
                                        fill_alpha=0.3))
        mod = self.modulation()
        if mod:
            ideal = IDEAL[mod]
            implot.plot_scatter(f"Идеальные точки {mod}", ideal.real.copy(), ideal.imag.copy(),
                                **style(marker=implot.Marker_.cross, marker_size=10,
                                        marker_line_color=IDEAL_COLOR, line_weight=2))
        implot.end_plot()

    def timing_plot(self, height):
        # Гистограмма момента отсчета: при хорошей синхронизации - один столбец
        if not implot.begin_plot("Момент отсчета в символе", imgui.ImVec2(-1, height),
                                 implot.Flags_.no_legend):
            return
        implot.setup_axes("Сдвиг от начала пакета, отсч.", "Пакетов",
                          0, implot.AxisFlags_.auto_fit)
        # Все возможные сдвиги (SPS // 2 ... SPS // 2 + SPS - 1), деления по целым
        first = self.sps // 2
        implot.setup_axis_limits(implot.ImAxis_.x1, first - 0.7, first + self.sps - 0.3,
                                 imgui.Cond_.always)
        implot.setup_axis_ticks(implot.ImAxis_.x1, [float(v) for v in range(first, first + self.sps)])
        if self.result is not None and len(self.result["taus"]):
            values, counts = np.unique(self.result["taus"], return_counts=True)
            implot.plot_bars("Пакетов", values.astype(np.float64), counts.astype(np.float64),
                             0.6, **style(fill_color=ACCENT, line_color=ACCENT))
        implot.end_plot()

    def errors_plot(self):
        # Ошибки по времени начала пакетов: та же ось, что у графика амплитуды
        mod = self.modulation()
        has = self.result is not None and len(self.result["starts"]) and mod
        title = f"Ошибки в пакетах ({mod})" if has else "Ошибки в пакетах (модуляция не определена)"
        if not implot.begin_plot(f"{title}###errors", imgui.ImVec2(-1, -1),
                                 implot.Flags_.no_legend):
            return
        implot.setup_axes("Время, мс", "Ошибок, бит", implot.AxisFlags_.auto_fit, 0)
        if has:
            errors = self.packet_errors(mod)
            implot.setup_axis_limits(implot.ImAxis_.y1, 0, max(1, errors.max()) * 1.15,
                                     imgui.Cond_.always)
            t = self.result["starts"] * 1e3 / FS
            implot.plot_bars("Ошибок", t, errors.astype(np.float64), BUFFER * 1e3 / FS,
                             **style(fill_color=BAD, line_color=BAD))
            # Область, видимая на графике амплитуды
            implot.plot_inf_lines("Окно", np.array(self.cur), **style(line_color=GREY))
        implot.end_plot()

    def placeholder(self):
        # Пояснение посередине пустой области графиков
        if self.receiver is not None:
            lines = (f"{fa.ICON_FA_RSS}  Ожидание данных на UDP {self.receiver.port}",
                     "Запустите ./main rx")
        else:
            lines = (f"{fa.ICON_FA_FOLDER_OPEN}  Нет сигнала",
                     "Выберите канал или файл слева")
        avail = imgui.get_content_region_avail()
        h = imgui.get_text_line_height_with_spacing()
        imgui.set_cursor_pos_y(imgui.get_cursor_pos_y() + (avail.y - h * len(lines)) / 2)
        for i, line in enumerate(lines):
            imgui.set_cursor_pos_x((avail.x - imgui.calc_text_size(line).x) / 2)
            if i == 0:
                imgui.text(line)
            else:
                imgui.text_disabled(line)

    def plots(self):
        if self.signal is None:
            self.placeholder()
            return
        self.amplitude_plot(imgui.get_content_region_avail().y * 0.4)
        # Ниже: квадратное созвездие слева, статистика синхронизации справа
        avail = imgui.get_content_region_avail()
        side = min(avail.y, avail.x * 0.6)
        imgui.begin_child("constellation", imgui.ImVec2(side, avail.y))
        self.constellation_plot()
        imgui.end_child()
        imgui.same_line()
        imgui.begin_child("stats", imgui.ImVec2(0, avail.y))
        self.timing_plot((avail.y - imgui.get_style().item_spacing.y) / 2)
        self.errors_plot()
        imgui.end_child()

    def gui(self):
        if not self.styled:
            setup_style()
            self.styled = True
        if self.receiver is not None:
            self.update_live()
        self.hotkeys()
        imgui.begin_child("settings", imgui.ImVec2(340, 0),
                          CHILD_BORDERS | imgui.ChildFlags_.resize_x)
        self.settings()
        imgui.end_child()
        imgui.same_line()
        imgui.begin_child("plots")
        self.plots()
        imgui.end_child()


def load_fonts():
    """Шрифт по умолчанию hello_imgui, но со всеми символами, включая кириллицу."""
    font_params = hello_imgui.FontLoadingParams()
    font_params.use_full_glyph_range = True
    font_params.merge_font_awesome = True
    hello_imgui.load_font("fonts/DroidSans.ttf", 15.0, font_params)


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
    params = hello_imgui.RunnerParams()
    params.callbacks.show_gui = app.gui
    params.app_window_params.window_title = title
    params.app_window_params.window_geometry.size = (1400, 850)
    # В live кадры рисуются всегда: по умолчанию без движения мыши
    # окно переходит на 10 кадров/с, и прокрутка дергается
    params.fps_idling.fps_idle = 0 if live else 10
    if OLD_FONTS:
        params.callbacks.load_additional_fonts = load_fonts
    addons = immapp.AddOnsParams()
    addons.with_implot = True
    immapp.run(params, addons)


if __name__ == "__main__":
    main()
