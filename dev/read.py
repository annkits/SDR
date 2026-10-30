import os
import sys

import numpy as np
import matplotlib.pyplot as plt

# Аргумент: канал down/up (файл dev/build/rx_<канал>.pcm) или путь к файлу
arg = sys.argv[1] if len(sys.argv) > 1 else "down"
if arg in ("down", "up"):
    name = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "build", f"rx_{arg}.pcm")
else:
    name = arg

# Сколько отсчетов показывать на графике
N = 2000000

# Формат файла: int16 little-endian I, Q, I, Q, ...
data = np.fromfile(name, dtype='<i2').astype(np.float32)
real = data[0::2]
imag = data[1::2]

complex_signal = real + 1j * imag
amplitude = np.abs(complex_signal)
print(f"Всего отсчетов: {len(amplitude)}, max = {amplitude.max():.0f}, "
      f"среднее = {amplitude.mean():.1f}")

# Окно вокруг самого сильного отсчета
start = max(0, int(np.argmax(amplitude)) - N // 2)
segment = amplitude[start:start + N]
count = np.arange(start, start + len(segment))

plt.figure(figsize=(12, 8))

plt.plot(count, real[start:start + N], linewidth=0.7, label='I')
plt.plot(count, imag[start:start + N], linewidth=0.7, label='Q')
plt.plot(count, segment, color='black', linewidth=0.7, label='|I + jQ|')
plt.title("I, Q и амплитуда")
plt.xlabel("Отсчеты")
plt.ylabel("Значение")
plt.legend()
plt.grid(True)

# ---------------- Символьная синхронизация BPSK ----------------
# Параметры как в main.cpp
SPS = 10                  # отсчетов на символ
BUFFER = 1920             # отсчетов в пакете
N_SYMBOLS = BUFFER // SPS
PREAMBLE = np.array([1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1], dtype=np.uint8)  # Баркер-13

# Преамбула в отсчетах: 0 -> -1, 1 -> +1, каждый символ SPS раз
template = np.repeat(2.0 * PREAMBLE - 1, SPS)

# 1. Поиск пакетов по энергии: порог посередине между шумом и сигналом
WIN = 64
env = np.convolve(amplitude, np.ones(WIN) / WIN, mode='same')
lo, hi = np.percentile(env, [5, 95])
above = env > (lo + hi) / 2
rises = np.flatnonzero(above[1:] & ~above[:-1]) + 1

MARGIN = 100              # запас вокруг начала пакета
packets = 0
taus = []
constellation = []
next_free = 0             # раньше этого отсчета новый пакет не ищем

for r in rises:
    if r < next_free:     # фронт внутри уже найденного пакета
        continue
    if r < MARGIN or r + BUFFER + MARGIN > len(complex_signal):
        continue
    if above[r:r + BUFFER].mean() < 0.8:  # выброс шума
        continue
    seg = complex_signal[r - MARGIN:r + BUFFER + MARGIN]

    # 2. Корреляция с преамбулой: пик - начало пакета
    corr = np.correlate(seg[:2 * MARGIN + len(template)], template, mode='valid')
    p = int(np.argmax(np.abs(corr)))
    gain = np.abs(corr[p]) / np.sum(template ** 2)  # амплитуда сигнала
    next_free = r - MARGIN + p + BUFFER

    # 3. Согласованный фильтр: импульс прямоугольный, фильтр - сумма SPS отсчетов
    mf = np.convolve(seg, np.ones(SPS), mode='full')

    # 4. Момент отсчета: сдвиг внутри символа с максимальной энергией.
    #    Окно сдвинуто на SPS // 2 из-за задержки фильтров
    start = p + SPS // 2
    blk = mf[start:start + N_SYMBOLS * SPS].reshape(N_SYMBOLS, SPS)
    tau = int(np.argmax(np.sum(np.abs(blk) ** 2, axis=0)))
    sym = blk[:, tau] / (gain * SPS)

    packets += 1
    taus.append(SPS // 2 + tau)
    constellation.append(sym)

if packets == 0:
    print("Пакеты не найдены")
else:
    print(f"Найдено пакетов: {packets}")
    # Распределение выбранного момента отсчета по пакетам
    values, counts = np.unique(taus, return_counts=True)
    print("Момент отсчета в символе: " +
          ", ".join(f"{v} - {c} пак." for v, c in zip(values, counts)))

    constellation = np.concatenate(constellation)
    plt.figure(figsize=(12, 5))

    plt.subplot(1, 2, 1)
    plt.scatter(constellation.real, constellation.imag, s=2, alpha=0.3)
    plt.title("Символы после синхронизации")
    plt.xlabel("I")
    plt.ylabel("Q")
    plt.axis('equal')
    plt.grid(True)

    plt.subplot(1, 2, 2)
    plt.hist(taus, bins=np.arange(min(taus), max(taus) + 2) - 0.5)
    plt.title("Момент отсчета")
    plt.xlabel("Отсчет в символе")
    plt.ylabel("Пакетов")
    plt.grid(True)

plt.show()
