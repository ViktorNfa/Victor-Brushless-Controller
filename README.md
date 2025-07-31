# Victor Brushless Controller (VBC)

This repository contains the **firmware** and **KiCad hardware design files** for the **Victor Brushless Controller (VBC)**, a custom BLDC motor controller inspired by the [Dagor Brushless Controller](https://github.com/byDagor/Dagor-Brushless-Controller).

## Overview

VBC is a compact, ESP32-based brushless controller designed for use in robotic and actuator applications. It integrates key hardware components to support precise control of brushless motors with wired or wireless communication.

The controller supports three operation modes:
- **Position control**.
- **Velocity control**.
- **Torque control**.

In both voltage- and current-control modes. It is compatible with the [SimpleFOC](https://docs.simplefoc.com/) library and can be programmed using the Arduino IDE.

## Features

- ESP32-based microcontroller.
- Onboard 3-phase MOSFET driver and half-bridges.
- Integrated 14-bit magnetic encoder.
- Current sensing resistors.
- Onboard temperature sensor.
- Custom firmware tailored for SimpleFOC.
- KiCad schematics and PCB layout included.

## Hardware Specs

| Specification             | Value           |
|---------------------------|------------------|
| Board Dimensions          | ~44 x 44 mm      |
| Supply Voltage            | 5 – 24 V         |
| Peak RMS Current          | up to 40 A       |
| Magnetic Sensor Resolution| 14 bits          |
| Temperature Range         | -10°C to 120°C   |

> 🧪 Note: This is a **custom derivative project**, still under development. Use with caution and test thoroughly.

## Repository Structure

- `VBC-SW/` – Source code for the controller (Arduino + SimpleFOC).
- `VBC-HW/` – KiCad project files for PCB schematics and layout.
- `KiCad_libraries/` – Custom footprints & symbols needed for KiCad.

## Acknowledgements

Based on the fantastic work by [@byDagor](https://github.com/byDagor) and the [Dagor Brushless Controller](https://docs.dagor.dev/).

## License

To decide – see `...` file for details.
