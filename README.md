# Elevator Safety & EV Access Control System

This Webots project simulates an autonomous safety monitoring system for an elevator lobby. We designed this system to address the safety risks associated with lithium-ion batteries in enclosed spaces. The goal is simple but technically complex: **allow humans to use the elevator, but actively block and lock out Electric Vehicles (Green EVs).**

Rather than just passively observing, our system implements **active defence behaviours**. We moved beyond simple obstacle avoidance to create a multi-agent system that can visually identify a threat, coordinate an alarm, and physically intercept the target.

---

## Project Overview: Intelligent Active Defence

Our solution relies on two robots working in tandem with a central building supervisor. The core innovation here is the shift from “Open Loop” patrolling to **“Closed Loop” Vision-based control**:

1. **Smart Access Control:**
   * **Human/Non-EV:** Authorised. When detected, the elevator doors automatically open.
   * **Electric Vehicle:** Restricted. When detected, the doors lock shut and the hallway alarms trigger.

2. **Layered Surveillance:**
   * **The Patrol Robot:** It doesn't just watch; it acts. Upon detecting an EV, it switches to **Active Pursuit**, physically chasing the vehicle to cut off its path to the elevator.
   * **The Stationary Guard:** A fixed camera providing a second angle, ensuring redundancy and cross-verifying alarms with the central system.

---

## Technical Architecture

We implemented the system using three distinct controllers:

### 1. The Patrol Robot (`epuck_fsm_controller.c`)

This is the most complex agent. We built a **Finite State Machine (FSM)** that prioritises visual tasks over basic navigation.

* **Patrol & Scan (Low Priority):** The robot navigates the hall using IR sensors. To solve the issue of blind spots, we added a state where the robot performs a **360° spin** every 800 steps.
* **Visual Servoing (High Priority):** We implemented a **Proportional Controller** for the pursuit logic. Instead of a simple “move towards green” command, the robot calculates the horizontal error (`CenterX - ImageCenter`) and adjusts motor speeds dynamically to smoothly track and intercept the target.
* **Blocking Logic:** Once the robot intercepts the target (Distance < 5 cm), it performs an emergency stop to act as a physical barrier.

### 2. The Stationary Guard (`epuck_camera_stationary.c`)

Acting as the “CCTV” of the system, this robot focuses on the elevator entrance.

* **Visual Processing:** It constantly analyses the scene for the specific green colour signature of the EV.
* **Communication:** Upon detection, it broadcasts a radio signal (Channel 2) to the Supervisor. This triggers the **Blue Wall Alarm**, providing a secondary safety layer independent of the patrol robot.

### 3. The Supervisor (`vpe_supervisor.py`)

This script acts as the “Building Management System.”

* **Door Logic:** It handles the mechanics of the centre-opening elevator doors (Open for Red, Lock for Green).
* **Zone Alarm (Red Light):** It monitors absolute GPS coordinates. If an EV crosses the “Safety Line” on the floor, the Red Wall Light triggers immediately.

---

## Visual & Logic Guide

We designed the visual feedback to be intuitive for testing:

| Component | Visual Indicator | Condition | Result |
| :--- | :--- | :--- | :--- |
| **Zone Alarm** |  **Red Wall Light** | EV crosses the yellow floor line. | Doors **CLOSE**. |
| **Camera Alarm** |  **Blue Wall Light** | Stationary Cam sends a detection signal. | Doors **CLOSE**. |
| **Active Block** |  **Patrol Pursuit** | Patrol Robot sees the Green EV. | Robot **CHASES & BLOCKS**. |
| **Access Grant** |  **Doors Open** | Red Non-EV is in the lobby. | Doors **OPEN**. |

---

## Hypothesis Testing & Data Logging

To properly evaluate our main hypothesis (**H1**) — *that high-priority visual tasks can reliably and rapidly suppress low-priority navigation tasks* — we realised we couldn't just rely on visual observation.

We implemented a dual-logging system that saves data to CSV files automatically:

| Log File | Source | What we are measuring |
| :--- | :--- | :--- |
| `h1_patrol_data.csv` | **Patrol Robot** | Logs the `State_Priority` (0 vs 1). We use this to prove the **Physical Reaction Time** — showing exactly how many milliseconds it takes for the motors to switch from “Wandering” to “Chasing” once green pixels are seen. |
| `h1_stationary_data.csv` | **Stationary Robot** | Logs `Pixels_Detected` vs `Signal_Sent`. We use this to prove **Detection Reliability**, verifying that the alarm signal is sent the instant the target is visible. |

*Note: These files are generated in the root directory when you quit the simulation.*

---

## Running the Simulation

1. Open `FinalProject/worlds/elevator_lobby.wbt` in **Webots**.
2. **Build** both C controllers (`epuck_fsm_controller` and `epuck_camera_stationary`).
3. Press **Play** (▶).
4. Use the keyboard controls (make sure to click inside the 3D view first):

| Key | Function |
| :--- | :--- |
| **↑ ↓ ← →** | Drive the **Green EV** (Triggers Alarms & Pursuit). |
| **W / A / S / D** | Move the **(Human) Non-EV** (Triggers Door Open). |
| **R** | **Reset** all vehicles to start. |
| **E** | Manually toggle Elevator Doors. |
| **T** | Test Alarm Lights sequence. |
| **Q** | **Quit** and save all CSV data. |

---

## Team Contribution

We divided the project workload based on our technical strengths, splitting the development of the Stationary Robot features between us to ensure they integrated smoothly with the rest of the system:

| Member | Focus Area | Key Implementation |
| :--- | :--- | :--- |
| **Yousuf H** | **Perception & Environment** | **Simulation & Vision Setup:**<br>Yousuf handled the simulation environment (`vpe_supervisor.py`), managing the elevator door logic and zone alarms. He also established the **Computer Vision foundation**, writing the color-thresholding algorithms used by both robots and programming the LED feedback patterns for the stationary unit. |
| **Nasrudin A** | **Architecture & Control** | **System Logic & Evaluation:**<br>Nasrudin took charge of the complex control architectures. This included building the **Visual Servoing (Proportional Control)** for the Patrol robot and implementing the **Inter-Robot Communication protocol** that allows the stationary camera to trigger remote alarms. He also designed the **Dual-CSV Data Logging system** to rigorously test Hypothesis H1. |
