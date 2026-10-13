# SDR — Производственная практика по программно-определяемому радио

Репозиторий содержит материалы производственной практики по работе с **Software-Defined Radio (SDR)** на базе **Adalm-Pluto**.

Основные направления:
- Работа с **GNU Radio Companion**
- Приём и анализ реальных радиосигналов (FM, Wi-Fi)
- Прямая работа с устройством через **SoapySDR** + поддержка **timestamp**
- Сборка и использование необходимых библиотек (libiio, libad9361, SoapySDR, SoapyPlutoSDR)

---

## Структура репозитория

| Папка / файл | Описание |
|--------------|----------|
| `lab1/` | Лабораторная работа №1: Построение FM-радиоприёмника в GNU Radio |
| `lab2/` | Лабораторная работа №2: Приём и визуализация Wi-Fi сигнала (2.4 ГГц) |
| `dev/` | Лабораторная работа №3: Работа с Adalm Pluto через SoapySDR + Timestamp |
| `SoapySDR/` | Исходники SoapySDR (субмодуль) |
| `SoapyPlutoSDR/` | Драйвер SoapyPlutoSDR с поддержкой timestamp (субмодуль) |
| `libiio/` | Библиотека libiio (субмодуль) |
| `libad9361-iio/` | Библиотека libad9361 (субмодуль) |

---

## Лабораторные работы

### Lab 1 — GNU Radio. Построение FM-приёмника
Сборка FM-радиоприёмника из блоков GNU Radio Companion без написания кода.  
Используется **PlutoSDR Source**, Low Pass Filter, WBFM Receive, Audio Sink.

→ [Отчёт](lab1/README.md)

### Lab 2 — Приём Wi-Fi сигнала
Настройка спектрального анализатора на диапазон 2.4 ГГц.  
Визуализация с помощью QT GUI Frequency Sink и Waterfall Sink.

→ [Отчёт](lab2/README.md)

### Lab 3 — Adalm Pluto SDR & Timestamp
Прямая работа с устройством на C++ через SoapySDR.  
Реализована синхронная передача/приём IQ-сэмплов с использованием временных меток (timestamp) с FPGA.  
Запись RX-буфера в `.pcm` и визуализация I/Q.

→ [Отчёт](dev/lab3_report.md)  
→ Код: [`dev/main.cpp`](dev/main.cpp), [`dev/read.py`](dev/read.py)

---

## Требования

- Adalm-Pluto SDR
- Ubuntu / Linux
- GNU Radio Companion
- CMake, g++, Python 3
- Библиотеки: SoapySDR, libiio, libad9361-iio, SoapyPlutoSDR (с поддержкой timestamp)

Инструкции по установке библиотек находятся в материалах лабораторных работ и в папках соответствующих субмодулей.

---

## Автор

Короткова Анна
Группа ИКС-433 
Производственная практика по SDR
