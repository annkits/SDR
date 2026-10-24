import numpy as np
import matplotlib.pyplot as plt

name = "/home/annkits/Рабочий стол/sdr/dev/build/txdata.pcm"

real = []
imag = []
count = []
counter = 0

with open(name, "rb") as f:
    index = 0
    while (byte := f.read(2)):
        if index % 2 == 0:
            real.append(int.from_bytes(byte, byteorder='little', signed=True))
            counter += 1
            count.append(counter)
        else:
            imag.append(int.from_bytes(byte, byteorder='little', signed=True))
        index += 1

real = np.array(real, dtype=np.float32)
imag = np.array(imag, dtype=np.float32)

complex_signal = real + 1j * imag
amplitude = np.abs(complex_signal)

plt.figure(figsize=(12, 8))

plt.plot(count, amplitude, color='red', label='|I + jQ|')
plt.title("Амплитуда")
plt.xlabel("Отсчеты")
plt.ylabel("Амплитуда")
plt.legend()
plt.grid(True)

plt.show()