# Rocket Flight Computer and Telemetry Analysis

a custom-built model rocket equipped with an onboard flight computer designed to acquire, record, and analyze flight data using inertial measurement and soon barometric pressure sensors.

<img width="1086" height="1448" alt="Rocket_on_rail_test2" src="https://github.com/user-attachments/assets/01aa0662-3469-46c8-98e0-5aad5920e72c" />


## 1. Project Overview

this project involves the design, construction, and testing of a model rocket equipped with a custom-developed flight computer. the onboard system uses an Arduino Nano ESP32 and an MPU6050 inertial measurement unit to collect flight data during launch and flight.

the primary objective is to develop an onboard data acquisition system capable of measuring the rocket's motion and estimating important flight characteristics, including altitude, velocity, acceleration, and orientation. recorded sensor data is subsequently processed using Python to evaluate flight performance, investigate measurement accuracy, and identify limitations in the estimation methods.

the project integrates mechanical design, computer-aided design (CAD), electronics, embedded programming, sensor calibration, and numerical data analysis.

## 2. Rocket Specifications

| Specification | Description |
|---|---|
| Initial Mass | 610 g |
| Propulsion | 4 × Estes C6-5 solid rocket motors |
| Flight Computer | Arduino Nano ESP32 |
| Inertial Measurement Unit | MPU6050 |
| Data Storage | microSD card |
| Embedded Programming | Arduino C++ |
| Data Processing | Python |

## 3. Project Objectives

- design and construct a functional model rocket using commercially manufactured solid rocket motors.
- develop a custom onboard flight computer for sensor data acquisition.
- implement accelerometer and gyroscope calibration procedures.
- record time-stamped sensor measurements to a microSD card.
- process recorded measurements to estimate altitude, velocity, acceleration, and rocket orientation.
- develop Python-based tools for telemetry processing, visualization, and flight performance analysis.
- evaluate sensor accuracy, measurement limitations, and sources of accumulated error.

## 4. Repository Structure

The repository is organized into six primary directories, each corresponding to a different stage or technical aspect of the project.

| Directory | Description |
|---|---|
| `analysis/` | python scripts, data processing algorithms, telemetry analysis, and flight data visualizations. |
| `cad/` | computer-aided design files, 3D models, and mechanical component designs. |
| `data/` | raw and processed flight data, including sensor measurements recorded during testing and flight. |
| `design/` | rocket design specifications, structural considerations, component layouts, and engineering documentation. |
| `firmware/` | Arduino Nano ESP32 source code, sensor configuration, calibration routines, and data acquisition software. |
| `hardware/` | electronic components, wiring diagrams, circuit layouts, and flight computer hardware documentation. |

## 5. Hardware Architecture

the onboard electronics consist of an Arduino Nano ESP32 interfaced with an MPU6050 accelerometer and gyroscope and a microSD card module.

the flight computer collects inertial and atmospheric measurements, processes sensor readings, and records time-stamped data for subsequent post-flight analysis.

## 6. Software and Data Processing

**Embedded Software**
- sensor initialization and configuration.
- accelerometer and gyroscope calibration.
- inertial measurement acquisition.
- data logging and microSD storage management.

**Post-Flight Analysis**
- python-based telemetry processing.
- sensor filtering and orientation estimation.
- altitude and velocity estimation.
- numerical integration and data processing.
- Matplotlib-based data visualization.
- evaluation of flight characteristics and sensor performance.

## 7. Experimental Results

recorded flight data is processed and visualized to evaluate the rocket's measured performance and the effectiveness of the onboard flight computer.

Analysis includes:
- acceleration versus time.
- velocity versus time.
- roll and pitch versus time.
- gyroscope measurements and rotational behavior.

detailed graphs, calculations, and interpretations are documented in the `analysis/` directory.

## 8. Future Improvements

- improve orientation estimation through sensor fusion algorithms.
- investigate gyroscope drift and accumulated measurement error.
- refine altitude and velocity estimation methods.
- improve sensor calibration and data acquisition reliability.
- evaluate additional filtering and estimation techniques.
- investigate real-time wireless telemetry capabilities.

---

**Project Status:** Ongoing

**Developed By:** Anthony Mueller
