# 🎮 ESP32-GBC: High-Performance Bare-Metal Game Boy & Game Boy Color Console

[![ESP32](https://img.shields.io/badge/Platform-ESP32-blue.svg)](https://www.espressif.com/)
[![Framerate](https://img.shields.io/badge/Framerate-35--45%2B%20FPS-brightgreen.svg)]()
[![PSRAM](https://img.shields.io/badge/PSRAM-None%20Required%20(195KB%20DRAM)-orange.svg)]()
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)]()

A high-performance, bare-metal hardware emulator for **Nintendo Game Boy (DMG)** and **Game Boy Color (CGB)** titles running on a standard **ESP32-WROOM-32** microcontroller **without external PSRAM**. 

Engineered with a dual-core asynchronous pipeline, 2-way set-associative virtual memory paging, MBC3 real-time clock synchronization, and custom S-curve color gamma correction.

---

## ⚡ Highlights & Engineering Features

* **No PSRAM Required**: Runs full 2MB Game Boy Color games entirely within the ESP32's internal $195\text{ KB}$ 8-bit DRAM budget with $\sim 49\text{ KB}$ of safe free headroom.
* **Dual-Core Asynchronous Engine**:
  * **Core 0**: Dedicated $100\%$ to Z80 CPU opcode execution, PPU scanline generation, and MBC memory banking.
  * **Core 1**: Concurrently handles $40\text{ MHz}$ display streaming and $20\text{ MHz}$ MicroSD sector streaming with zero-lock semaphore synchronization.
* **2-Way Set-Associative 1KB Page Cache**: $24\text{ KB}$ dynamic cache with LRU replacement policy eliminates sprite decompression thrashing (e.g. *Pokémon Red/Blue*).
* **Full MBC3 Real-Time Clock (RTC) Support**: Accurately emulates day/hour/minute/second counters and latching protocols (e.g. *Pokémon Gold/Silver/Crystal* time events).
* **Dual Independent SPI Buses**: Hardware **VSPI (SPI3)** dedicated to the display and **HSPI (SPI2)** dedicated to the MicroSD card for zero bus contention.
* **S-Curve Gamma & Contrast Correction**: Eliminates backlit LCD glare and inverts Little-Endian/Big-Endian subpixel channels for rich, authentic retro colors.
* **Live In-Game Color Profile Switcher**: Toggle between 4 hardware color profiles on the fly by pressing **START + SELECT**.
* **Non-Volatile Save System**: Automatically flushes dirty Cartridge RAM to `.sav` files on the MicroSD card.

---

## 🛠️ Hardware Requirements

| Component | Specification |
| :--- | :--- |
| **Microcontroller** | ESP32-WROOM-32 / ESP32 Dev Module (Dual-Core 240 MHz) |
| **Display** | 2.8" or 2.4" ILI9341 SPI TFT ($240 \times 320$, 3.3V) |
| **Storage** | MicroSD Card Slot / Module (SPI Mode) |
| **Controls** | 8x Momentary Tactile Pushbuttons (Active LOW) |
| **Power** | 5V USB-C or 3.7V LiPo Battery with 3.3V LDO |

---

## 🔌 Hardware Pinout & Wiring

```
       ESP32-WROOM-32                       2.8" ILI9341 SPI TFT (VSPI)
  ┌──────────────────────┐                 ┌────────────────────────────┐
  │ GPIO 23 (VSPI MOSI)  │────────────────>│ MOSI / SDI (40 MHz SPI)    │
  │ GPIO 18 (VSPI SCK)   │────────────────>│ SCK / CLK  (40 MHz Clock)  │
  │ GPIO 15 (TFT CS)     │────────────────>│ CS                         │
  │ GPIO 2  (TFT DC)     │────────────────>│ DC / RS (0=Cmd, 1=Data)    │
  │ GPIO 4  (TFT RST)    │────────────────>│ RESET                      │
  │ +3V3                 │─────────────────│ VCC (or 5V if module LDO)  │
  │ GND                  │─────────────────│ GND                        │
  │ +3V3                 │─────────────────│ LED / Backlight            │
  └──────────────────────┘                 └────────────────────────────┘

       ESP32-WROOM-32                         MicroSD Socket (HSPI)
  ┌──────────────────────┐                 ┌────────────────────────────┐
  │ GPIO 5  (HSPI MOSI)  │────────────────>│ CMD / MOSI (Pin 3)         │
  │ GPIO 19 (HSPI MISO)  │<────────────────│ DAT0 / MISO (Pin 7)        │
  │ GPIO 21 (HSPI SCK)   │────────────────>│ CLK / SCK  (Pin 5) (20MHz) │
  │ GPIO 22 (SD CS)      │────────────────>│ CD/DAT3/CS (Pin 2)         │
  │ +3V3                 │──────┬──────────│ VDD (Pin 4)                │
  │ GND                  │──────┼──────────│ VSS (Pin 6)                │
  └──────────────────────┘      │          └────────────────────────────┘
                              [100nF]
                                │
                               GND

       ESP32 GPIO                           8 Tactile Switches (Active LOW)
  ┌──────────────────────┐                 ┌──────────────┐
  │ GPIO 13 (UP)         │────────────────o│ D-Pad UP     │o─── GND
  │ GPIO 12 (DOWN)       │────────────────o│ D-Pad DOWN   │o─── GND (*No ext pullup)
  │ GPIO 14 (LEFT)       │────────────────o│ D-Pad LEFT   │o─── GND
  │ GPIO 27 (RIGHT)      │────────────────o│ D-Pad RIGHT  │o─── GND
  │ GPIO 32 (BUTTON A)   │────────────────o│ Button A     │o─── GND
  │ GPIO 33 (BUTTON B)   │────────────────o│ Button B     │o─── GND
  │ GPIO 25 (START)      │────────────────o│ Button START │o─── GND
  │ GPIO 26 (SELECT)     │────────────────o│ Button SELECT│o─── GND
  └──────────────────────┘                 └──────────────┘
```

> **⚠️ Critical Strapping Pin Note:** `GPIO 12` is used for `D-Pad DOWN`. **Do NOT place an external pull-up resistor on GPIO 12**, as pulling it HIGH during power-on forces internal flash voltage to 1.8V and bricks boot. Internal weak pull-ups are enabled safely in software after boot.

---

## ⚙️ Arduino IDE Compilation Settings

1. Install **ESP32 Board Package** (`esp32` by Espressif Systems v2.x or v3.x).
2. Install Required Libraries:
   * **`Adafruit_GFX`**
   * **`Adafruit_ILI9341`**
   * **`SD`** (Built-in ESP32 SD library)
3. Set **Tools** menu options:
   * **Board**: `"ESP32 Dev Module"`
   * **CPU Frequency**: `"240MHz (WiFi/BT)"`
   * **Flash Frequency**: `"80MHz"`
   * **Flash Mode**: `"QIO"`
   * **Partition Scheme**: `"Huge APP (3MB No OTA/1MB SPIFFS)"`
   * **Upload Speed**: `"921600"` (or `"256000"`)

---

## 🎮 How to Play

1. Format your MicroSD card as **FAT32**.
2. Copy your uncompressed `.gb` and `.gbc` ROM files to the root of the SD card.
3. Insert the SD card and power on the ESP32.
4. Use the **D-Pad** to scroll through the file picker and press **A** to launch.
5. In-Game Hotkeys:
   * **START + SELECT**: Cycle through 4 live subpixel color modes (Default: Mode 3 with S-Curve Contrast & Glare Reduction).

---

## 🧠 Memory Architecture Breakdown

```
 Total ESP32 Internal DRAM: ~195 KB
 ├── gb_s Core Engine       :  49.9 KB (Z80 registers, VRAM, WRAM, HRAM, OAM)
 ├── Native Framebuffer     :  45.0 KB (160 x 144 x 2 bytes 16-bit color)
 ├── Bank 0 ROM             :  16.0 KB (Permanently resident in fast RAM)
 ├── 2-Way Page Cache       :  24.0 KB (24 dynamic 1KB slots for Banks 2+)
 ├── Cartridge Save RAM     :  32.0 KB (Non-volatile SRAM for Pokémon saves)
 ├── FreeRTOS Task Stacks   :  13.0 KB (Render, SD, and Emulator tasks)
 └── Untouched Headroom     :  49.4 KB (Zero memory fragmentation / leaks)
```

---

## 📄 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
