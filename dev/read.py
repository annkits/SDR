import os
import sys

import numpy as np
import matplotlib.pyplot as plt

arg = sys.argv[1] if len(sys.argv) > 1 else "down"
if arg in ("down", "up"):
    name = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "build", f"rx_{arg}.pcm")
else:
    name = arg

N = int(sys.argv[2]) if len(sys.argv) > 2 else 100000

data = np.fromfile(name, dtype='<i2').astype(np.float32)
real = data[0::2]
imag = data[1::2]

complex_signal = real + 1j * imag
amplitude = np.abs(complex_signal)
print(f"Всего отсчетов: {len(amplitude)}, max = {amplitude.max():.0f}, "
      f"среднее = {amplitude.mean():.1f}")

first = int(np.argmax(amplitude > amplitude.max() / 2))
start = max(0, first - N // 20)
count = np.arange(len(amplitude))

plt.figure(figsize=(12, 8))

plt.plot(count, amplitude, color='red', linewidth=0.8, label='|I + jQ|')
plt.xlim(start, min(start + N, len(amplitude)))
plt.title("Амплитуда")
plt.xlabel("Отсчеты")
plt.ylabel("Амплитуда")
plt.legend()
plt.grid(True)

plt.show()
