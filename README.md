# Non-Invasive Elderly Fall Detection System

## Overview
This repository contains the firmware, hardware schematics, and mechanical design files for a privacy-preserving, non-invasive bathroom fall detection unit. 

Designed specifically to operate without the use of cameras, this system ensures complete user privacy in sensitive environments. It achieves high-accuracy fall detection by integrating a 60GHz mmWave radar sensor with an ESP32-S3 microcontroller, complemented by a verbal cancellation logic mechanism to rigorously filter false positive alerts.

## Key Features
* **Privacy-First Sensing:** Utilizes 60GHz mmWave radar, eliminating the need for optical cameras.
* **Edge Processing:** All signal processing and fall detection algorithms are executed locally on the ESP32-S3.
* **Verbal Cancellation Logic:** Integrates audio-based confirmation to prevent unnecessary emergency alerts in the event of a false trigger.
* **Custom Enclosure:** 3D-printable housing designed for optimal sensor positioning and environmental protection.

## System Hardware
* **Microcontroller:** ESP32-S3
* **Radar Sensor:** SEN0623 (60GHz mmWave)
* **[Add Power Supply/Battery module here]**
* **[Add Audio input/mic module here if separate from ESP32]**

## Hardware Visuals

### Printed Enclosure
![Fully Assembled 3D Printed Enclosure](path/to/your/enclosure_image.png)

### PCB Assembly
![Soldered PCB Configuration](path/to/your/soldered_pcb_image.png)

## Repository Structure
```text
elderly-fall-detection/
├── Firmware/       # ESP32-S3 source code and build files
├── Hardware/       # Schematics, PCB layouts, and BOM
├── Mechanical/     # STL files for 3D printing the enclosure
└── Docs/           # Final engineering design project report and diagrams