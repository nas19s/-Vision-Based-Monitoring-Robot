# Elevator Safety Monitoring Robot

This is a Webots simulation project aimed at creating a safety system to prevent electric vehicles (EVs) from entering restricted zones near elevators. We built a sophisticated, two-part system focusing on **resilient, high-priority reactive control**, which is crucial because EVs with lithium batteries pose a significant fire risk in enclosed spaces.

The e-puck robot acts as a stationary monitor, demonstrating an intelligent, high-priority safety response.

---

## Project Goal: Redundancy and Speed

Our primary technical challenge was achieving reliable detection and *immediate* reaction, even under noisy conditions. We solved this by implementing two independent detection methods that can each trigger the high-priority alarm:

1.  **Vision Detection (C-Controller):** The e-puck uses its on-board camera to process images and visually identify the EV's bright green safety marker.
2.  **Zone Monitoring (Python Supervisor):** A supervisor program acts as an authoritative, external position check, verifying if the EV's exact coordinates violate the zone boundary.

If *either* system detects a violation, the robot flashes its LEDs, and an external wall alarm is activated.

---

## Technical Architecture Overview

The true complexity of our work lies in the multi-controller setup, which forced us to integrate two different programming languages and control domains:

* **C Controller (`epuck_fsm_controller.c`):** This runs the e-puck's internal brain. It's responsible for all **real-time sensing (Camera processing)**, the **Finite State Machine (FSM)** logic, and controlling the robot’s actuators (LEDs).
* **Python Supervisor (`vpe_supervisor.py`):** This program has **supervisor privileges**, allowing it to manipulate the world. It calculates precise coordinates, implements the stabilising **Hysteresis** logic for the boundary, controls the EV's movement, and flashes the **Wall Alarm Light**.
* **Communication:** We used the Webots **Emitter/Receiver** radio devices (Channel 1) for the Python supervisor to send a quick, authoritative alarm signal to the C controller.

---

## Core Logic and Evaluation

To prove our hypothesis that a high-priority system can respond immediately, we implemented tight control logic and rigorous data logging.

### 1. FSM and Priority Control

The e-puck uses a three-state FSM to manage its response priority. This structure ensures that the alarm can instantly override any lower-priority state.

| State | Purpose | Logic |
| :--- | :--- | :--- |
| **MONITORING** | Idle state; robot is simply watching. | Single status LED on. |
| **TARGET\_DETECTED** | **Confirmation State:** A 96ms delay (3 time steps) to confirm the detection is stable. | Half of the LEDs are lit. |
| **ALARM\_ACTIVE** | The high-priority state; immediate and persistent until the threat is clear. | All LEDs flash rapidly. |

### 2. Camera Logic (C)

To prevent false alarms from shadows or slightly greenish walls, we engineered a very strict color filter:

* We only accept pixels where the Green channel is **at least twice** the Red and Blue values.
* The pixel count must exceed a threshold of **20** pixels to be considered a valid detection.

### 3. Edge Case Testing (New)

A key part of our project was testing the system's limitations. We created a special function (`E` key) in the supervisor to automatically run **systematic edge case tests**. This moves the EV to set positions and logs data on the camera's detection reliability at various distances.

---

## Experiment Data & Hypothesis (H1)

Our project tested the following hypothesis:

> A high-priority alarm behaviour triggered by visual or position detection can **reliably** and **rapidly** suppress low-priority monitoring behaviours, enabling a precise response within one control cycle.

Our data logging verifies this conclusion:

| Data Log File | What It Shows | Key Finding |
| :--- | :--- | :--- |
| `h1_experiment_data.csv` | **Response Time** | The robot consistently achieved a response time of **$32.000\text{ms}$** (one control step), confirming maximum speed. |
| `h1_vpe_data_[ts].csv` | **Zone Validation** | Records the precise time and position the EV crossed the boundary, validating the VPE's high reliability. |
| `h1_edge_cases_[ts].csv` | **Camera Limitations** | Provides the data necessary to analyse the detection range and sensitivity of the Camera logic. |

---

## Running the Simulation

1.  Open `FinalProject/worlds/elevator_lobby.wbt` in **Webots**.
2.  **Build** the C controller.
3.  Press **Play** (▶).
4.  Use the controls below to test the system.

| Key | Function |
| :--- | :--- |
| **↑↓←→** | Drive the Electric Vehicle. |
| **R** | Reset EV to starting position. |
| **E** | **Run Systematic Edge Case Test** (automated testing). |
| **T** | Test the wall alarm light. |
| **S** / **Q** | Save all data logs (S) or Quit and Save (Q). |

---

## Team Contribution Summary

| Member | Focus Area | Key Implementation |
| :--- | :--- | :--- |
| **Yousuf H** | Virtual Perception & Environment | `vpe_supervisor.py` (VPE logic, Hysteresis, Emitter, Wall Alarm Light control). |
| **Tairui Z** | Robot Control & Sensory Processing | `epuck_fsm_controller.c` (C structure, Camera image processing, Color thresholds, LED control). |
| **Nasrudin A** | Architecture & Evaluation | FSM Design and implementation, creation of **all CSV data logging systems** and the **Edge Case Test** framework. |
