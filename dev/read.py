import os
import sys

import numpy as np
import matplotlib.pyplot as plt

# Аргумент: канал down/up (файл dev/build/rx_<канал>.pcm) или путь к файлу,
# по умолчанию - down
arg = sys.argv[1] if len(sys.argv) > 1 else "down"
if arg in ("down", "up"):
    name = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "build", f"rx_{arg}.pcm")
else:
    name = arg

# Сколько отсчетов показывать: период "вкл/выкл" = 3840 отсчетов,
# на всем файле прямоугольники сливаются в сплошную полосу
N = 2000000

# Файл: чередующиеся int16 little-endian I, Q, I, Q, ...
data = np.fromfile(name, dtype='<i2').astype(np.float32)
real = data[0::2]
imag = data[1::2]

complex_signal = real + 1j * imag
amplitude = np.abs(complex_signal)
print(f"Всего отсчетов: {len(amplitude)}, max = {amplitude.max():.0f}, "
      f"среднее = {amplitude.mean():.1f}")

# Начинаем с самого сильного отсчета, чтобы не смотреть на шум до старта TX
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

plt.show()
