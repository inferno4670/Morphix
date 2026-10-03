# ⌚ MORPHIX
### Modular Wearable Health & Activity Platform

> **A customizable wearable platform built around sensing, embedded intelligence, modular hardware, and an adaptable physical design.**

![Status](https://img.shields.io/badge/Status-Prototype-orange)
![Platform](https://img.shields.io/badge/Platform-ESP32--S3-blue)
![Firmware](https://img.shields.io/badge/Firmware-Arduino%20C%2FC%2B%2B-green)
![Connectivity](https://img.shields.io/badge/Connectivity-Wi--Fi%20%2B%20BLE-purple)
![Hardware](https://img.shields.io/badge/Hardware-Modular-red)

---

## 🧬 What is MORPHIX?

**MORPHIX** is a modular wearable technology platform designed to combine physiological sensing, motion tracking, environmental sensing, user interaction, and audio into a compact wearable system.

Rather than treating the wearable as a fixed-purpose smartwatch, MORPHIX is designed around a **customizable architecture**. Hardware modules, firmware functionality, and the physical enclosure can evolve as the application changes.

### The core idea

```text
              ┌─────────────────────┐
              │       MORPHIX       │
              │   Wearable Core     │
              └──────────┬──────────┘
                         │
        ┌────────────────┼────────────────┐
        │                │                │
        ▼                ▼                ▼
   HEALTH DATA       MOTION DATA      INTERACTION
   MAX30102          MPU6050          TFT + Buttons
   BMP280            Activity         User Interface
        │                │                │
        └────────────────┼────────────────┘
                         │
                         ▼
                 ESP32-S3 PROCESSING
                         │
              ┌──────────┴──────────┐
              ▼                     ▼
           Wireless               Audio
          Wi-Fi + BLE       INMP441 + MAX98357A
```

---

## ✨ Key Features

### ❤️ Physiological Monitoring
- Heart-rate sensing using **MAX30102**
- SpO₂ estimation using **MAX30102**
- Wrist-oriented sensor placement
- Real-time sensor data processing

### 🏃 Motion & Activity
- **MPU6050** accelerometer + gyroscope
- Motion tracking
- Activity-related processing
- Gesture-oriented interaction potential
- Step/activity features

### 🌡️ Environmental Sensing
- **BMP280**
- Temperature sensing
- Atmospheric pressure sensing

> **Note:** BMP280 measures atmospheric pressure, not blood pressure.

### 🖥️ Wearable Interface
- 1.8-inch TFT display
- Four tactile buttons
- Custom embedded user interface
- Designed for direct interaction without requiring a phone for basic operation

### 🎙️ Audio
- **INMP441** digital microphone
- **MAX98357A** I²S audio amplifier
- Mini speaker
- Audio-ready hardware architecture

### 📡 Wireless
- ESP32-S3 platform
- Wi-Fi connectivity
- BLE capability
- Designed for future companion-device integration

---

## 🧩 Hardware

| Component | Role |
|---|---|
| **Seeed Studio XIAO ESP32-S3** | Main processing + wireless controller |
| **MAX30102** | Heart-rate + SpO₂ sensing |
| **MPU6050** | Accelerometer + gyroscope |
| **BMP280** | Temperature + atmospheric pressure |
| **1.8" TFT LCD** | Visual interface |
| **4 × Tactical Switches** | User input |
| **INMP441** | Digital microphone |
| **MAX98357A** | I²S audio amplifier |
| **Mini Speaker** | Audio output |
| **3.7V 1000mAh Li-ion Battery** | Portable power |

---

## 🔌 Current Hardware Architecture

MORPHIX currently uses the **XIAO ESP32-S3** as its main controller.

### I²C bus

```text
XIAO ESP32-S3
     │
     ├── SDA → D4
     └── SCL → D5
          │
          ├── MAX30102 → 0x57
          ├── MPU6050  → 0x68
          └── BMP280   → 0x76
```

### TFT interface

```text
TFT
 ├── SCK  → D8
 ├── MOSI → D10
 ├── CS   → D2
 ├── DC   → D1
 └── RST  → D0
```

### User controls

```text
UP     → D9
DOWN   → D3
BACK   → D6
SELECT → D7
```

---

## 🔊 Audio Architecture

```text
                 ┌──────────────┐
                 │  ESP32-S3    │
                 └──────┬───────┘
                        │
                     I²S BUS
              ┌─────────┴─────────┐
              │                   │
              ▼                   ▼
        ┌───────────┐       ┌───────────┐
        │ INMP441   │       │ MAX98357A │
        │ Microphone│       │ Amplifier │
        └───────────┘       └─────┬─────┘
                                  │
                                  ▼
                            Mini Speaker
```

---

## 🔋 Power

MORPHIX uses a **3.7V 1000mAh Li-ion battery** in the current prototype.

Approximate supplied dimensions:

**55 × 36 × 5 mm**

> Battery dimensions, protection circuitry, charging behavior, connector orientation, and final mechanical retention must be verified before production.

---

## 🧱 The MORPHIX Philosophy

The most important design principle is **customizability**.

MORPHIX is intended to evolve through three layers:

```text
┌───────────────────────────────────────┐
│           MORPHIX PLATFORM            │
├───────────────────────────────────────┤
│                                       │
│  1. HARDWARE                          │
│     Sensors / audio / interfaces      │
│                                       │
│  2. SOFTWARE                          │
│     Firmware / UI / data processing   │
│                                       │
│  3. PHYSICAL DESIGN                   │
│     Case / buttons / sensor windows   │
│                                       │
└───────────────────────────────────────┘
```

The goal is to allow the platform to be adapted for different applications without redesigning the entire concept from scratch.

---

## 🧠 MORPHIX Custom PCB — V2

The next hardware generation moves toward a **MORPHIX Core PCB**.

The Core PCB is intended to provide standardized interfaces for interchangeable modules rather than permanently locking every sensor onto one board.

```text
                 MORPHIX CORE PCB
                       │
          ┌────────────┼────────────┐
          │            │            │
          ▼            ▼            ▼
      SENSOR 1      SENSOR 2      AUDIO
      MODULE        MODULE        MODULE
          │            │            │
      MAX30102       BMP280      INMP441
      / other        / other     MAX98357A
                                  │
                               Speaker

                       │
                       ▼
                  DISPLAY PORT
                       │
                      TFT

                       │
                       ▼
                  EXPANSION PORT
```

### Design goal

**The custom PCB should support modularity, not eliminate it.**

The initial custom-board strategy therefore keeps the **XIAO ESP32-S3 as a removable controller module** while providing cleaner interfaces, power distribution, connectors, and mechanical integration.

---

## 🛠️ Development Stack

### Firmware
- Arduino IDE
- C/C++
- ESP32 Arduino framework

### Communication
- I²C
- SPI
- I²S
- Wi-Fi
- BLE
- NTP time synchronization

### Hardware
- XIAO ESP32-S3
- Embedded sensors
- TFT display
- Li-ion battery
- Modular connectors
- Custom PCB development

### Mechanical
- Custom wearable enclosure
- 3D-printable case development
- Component-specific mounting and clearances

---

## 📐 Prototype Mechanical Reference

| Component | Supplied dimension |
|---|---:|
| XIAO ESP32-S3 | 21 × 17.8 mm |
| TFT module | 38 × 60 × 10 mm |
| Li-ion battery | 55 × 36 × 5 mm |
| Tactical switch | 12 × 12 × 7.3 mm |
| MPU6050 module | 20 × 16 mm |
| INMP441 | 14 × 14 × 1 mm |
| Mini speaker | 24 × 15 × 4 mm |
| BMP280 IC/package | 2.5 × 2.5 × 0.93 mm* |
| MAX30102 IC/package | approximately 5.6 × 2.8 × 1.2 mm* |

\* Package-level dimensions are not necessarily the dimensions of the breakout modules. Final mechanical design must use verified footprint/module dimensions.

---

## 📁 Suggested Repository Structure

```text
MORPHIX/
│
├── firmware/
│   ├── MORPHIX.ino
│   ├── sensors/
│   ├── display/
│   ├── buttons/
│   ├── audio/
│   └── connectivity/
│
├── hardware/
│   ├── pcb/
│   ├── schematic/
│   ├── gerbers/
│   └── bom/
│
├── mechanical/
│   ├── case/
│   ├── drawings/
│   └── dimensions/
│
├── mobile/
│   └── companion-app/
│
├── documentation/
│   ├── architecture/
│   ├── pinout/
│   └── testing/
│
└── README.md
```

---

## 🚧 Development Status

| Area | Status |
|---|---|
| ESP32-S3 controller | 🟢 Prototype |
| TFT interface | 🟢 Prototype |
| MAX30102 | 🟢 Prototype |
| MPU6050 | 🟢 Prototype |
| BMP280 | 🟢 Prototype |
| Buttons | 🟢 Prototype |
| Battery integration | 🟡 Requires final validation |
| Audio subsystem | 🟡 Integration stage |
| Custom Core PCB | 🟡 Rev A development |
| Modular PCB interfaces | 🟡 Development |
| Final enclosure | 🟡 Development |
| Companion application | 🟡 Development / planned features |
| Production hardware | ⚪ Not released |

---

## ⚠️ Prototype Disclaimer

MORPHIX is an **engineering prototype and wearable technology project**.

Sensor outputs such as heart rate and SpO₂ are intended for experimentation, demonstration, and development. They should **not** be treated as medical-grade measurements or used for diagnosis or treatment.

Battery, charging, enclosure, thermal behavior, electrical safety, RF performance, and mechanical reliability must be validated before deployment outside controlled prototype use.

---

## 🎯 Project Vision

MORPHIX is built around one idea:

> **Don't build one wearable. Build a wearable platform that can evolve.**

```text
              MORPHIX
                 │
       ┌─────────┼─────────┐
       ▼         ▼         ▼
    SENSE      PROCESS    ADAPT
       │         │         │
    Sensors    ESP32-S3   Modules
       │         │         │
       └─────────┼─────────┘
                 ▼
          CUSTOM WEARABLE
```

---

### 🔧 Built with curiosity, solder, firmware, and an unreasonable number of wires.
