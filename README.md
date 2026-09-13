# ESP32 Safety-Oriented Thermal Management System

A closed-loop thermal controller built on dual-core FreeRTOS. The system regulates enclosure temperatures using an active PI controller with asymmetric Feed-Forward setpoint tracking, backed by an independent passive safety architecture derived from functional safety concepts (IEC 61508 / ISO 26262).

---

## Overview & System Design

This project demonstrates how to implement functional safety principles in a laboratory prototype without industrial compliance overhead. 

The system relies on a **layered defense strategy**:
1. **Physical Layer (Passive Safety):** The cork enclosure is sized so that maximum heat buildup never exceeds $T_{\text{env}} + 19^\circ\text{C}$. If the microcontroller hangs, sensor fails, or power drops, thermal dissipation inherently keeps all internal electronics below their $80^\circ\text{C}$ safe operating limit.
2. **Software Layer (Active Control):** The ESP32 executes a multi-rate FreeRTOS architecture. Core 1 runs the 20 Hz control loop and setpoint ramping engine, using Zero-Order Hold (ZOH) gating to process 0.5 Hz DHT22 readings safely. Core 0 handles network telemetry independently—ensuring Wi-Fi latencies never freeze the thermal control pipeline.

---

## Key Features

* **Passive Physical Safety Boundary:** Guaranteed hardware safety under total active-system failure ($T_{\text{max}} < 80^\circ\text{C}$).
* **Dual-Core Determinism:** Core-pinned FreeRTOS tasks scheduled with `vTaskDelayUntil()` to eliminate timing jitter.
* **Asymmetric Feed-Forward PI Control:** Tailored control logic accounting for single-direction forced-convection cooling (fan) vs. passive heating.
* **Multi-Rate Processing (20 Hz / 0.5 Hz):** 20 Hz profile/PWM evaluation with gated integral accumulation on fresh 0.5 Hz sensor frames.
* **Fault Tolerance & Safe-State Execution:** Hardware Watchdog Timer (10s WDT), thread-safe mutex memory protection (`xMutexRAM`), and automated fallback to 100% fan duty cycle (`STATE_FAULT`).
* **Non-Blocking UDP Telemetry:** Streams real-time temperature, setpoint, and actuator states over Wi-Fi with complete network fault isolation.

---

## System Hardware & Task Architecture

* **Microcontroller:** ESP32 Development Module
* **Temperature Sensor:** DHT22 / AM2302 (GPIO 4)
* **Cooling Actuator:** 5V DC Fan driven via N-Channel MOSFET (25 kHz PWM on GPIO 18)
* **Heat Source:** Constant 15 W thermal source in a manual-cut cork enclosure

| Task Name | Core | Priority | Frequency | Key Responsibility |
| :--- | :---: | :---: | :---: | :--- |
| `TaskThermalControl` | 1 | 3 (High) | 20 Hz | Sensor validation, FSM setpoint ramping, PI computation, PWM output, WDT reset |
| `TaskTelemetry` | 0 | 2 (Med) | 1 Hz | Thread-safe data copying, UDP packet formatting, Wi-Fi transmission, WDT reset |
