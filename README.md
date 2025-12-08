# Vision-Based High-Priority Reactive Monitoring Robot
### Application: Elevator Safety & EV Access Control System


This Webots project simulates an autonomous safety monitoring system for an elevator lobby. We designed this system to address the specific fire safety risks associated with lithium-ion batteries in enclosed spaces. The goal is simple in concept but technically complex to execute: **allow humans to use the elevator, but actively block and lock out Electric Vehicles (Green EVs).**

We moved beyond simple "obstacle avoidance" to create a multi-agent system capable of **Feature-Based Recognition**. A key focus of our research was ensuring the robot is "smart" enough to distinguish between a genuine threat (a moving car) and a false positive (a green pillar or wall), solving a common limitation in colour-based robotics.

---

## Project Overview: Intelligent Active Defence

Our solution relies on two robots working in tandem with a central building supervisor. The core of the project is the **Patrol Robot**, which we engineered to handle real-world uncertainty using a sophisticated vision pipeline.

1.  **Smart Access Control:**
    *   **Human/Non-EV:** Authorised. The elevator doors open automatically.
    *   **Electric Vehicle:** Restricted. The robot locks onto the target and physically intercepts.
    *   **Green Pillars/Walls:** **IGNORED.** We implemented a validation layer that identifies these as structural elements rather than vehicles, ensuring the robot does not get stuck attacking a wall.

2.  **Layered Surveillance:**
    *   **Patrol Robot:** Uses **Braitenberg Vehicle** logic for smooth, organic navigation and a **Finite State Machine** for threat assessment.
    *   **Stationary Guard:** A fixed camera providing redundant CCTV coverage.

---

## Technical Architecture

We implemented the system using three distinct controllers, with the bulk of the "intelligence" residing in the patrol unit.

### 1. The Patrol Robot (`epuck_fsm_controller.c`)

This is the primary agent. We implemented a **Finite State Machine (FSM)** integrated with a custom computer vision algorithm written in C.

*   **Vision Pipeline: "Spatial Context Validation"**
    We realised that relying solely on colour thresholding would lead to failure in a real environment. To fix this, we wrote a multi-stage filter:
    1.  **Chassis Detection:** Identifies the "Centre of Mass" of green pixels.
    2.  **Wheel Search:** Scans specifically for high-density dark pixels in the lower 30% of the object.
    3.  **Dynamic Revalidation (The "Force Release" Logic):**
        This is our key contribution to reliability. The robot continuously re-evaluates the target *during* the pursuit. If the target loses its vehicle characteristics (e.g., if the robot gets closer and realises the object has **Zero Black Pixels**), it triggers a **Force Release**. The robot immediately aborts the pursuit and resumes patrolling.

*   **Navigation: Braitenberg Algorithm**
    For patrol movement, we replaced simple "stop-and-turn" logic with a **Weighted Braitenberg Matrix**. By calculating motor speeds based on a weighted sum of all 8 distance sensors, the robot achieves smooth, curved trajectories around obstacles.

*   **State Machine Logic:**
    *   `STATE_VALIDATING`: The robot slows down to gather 10 frames of structural data before committing to a chase.
    *   `STATE_PURSUIT`: Uses **Visual Servoing** (Proportional Control) to steer towards the target.
    *   `STATE_ALARM_BLOCK`: Physical interception upon contact.
    *   `STATE_AVOIDING`: A priority interrupt state that handles immediate collision risks before returning the robot to its previous task.

### 2. The Stationary Guard (`epuck_camera_stationary.c`)

Acting as the "CCTV" of the system, this robot focuses on the elevator entrance.

*   **Visual Processing:** It acts as a tripwire, analysing the scene for the specific EV colour signature.
*   **Communication:** Upon detection, it broadcasts a radio signal (Channel 2) to the Supervisor to trigger the **Blue Wall Alarm**.

### 3. The Supervisor (`vpe_supervisor.py`)

This script acts as the "Building Management System."

*   **Door Logic:** Handles the mechanics of the centre-opening elevator doors (Open for Red, Lock for Green).
*   **Zone Alarm:** Monitors absolute GPS coordinates to trigger the Red Wall Light if the safety line is crossed.

---

## Visual & Logic Guide

We designed the visual feedback to be intuitive for testing:

| Component | Visual Indicator | Condition | Result |
| :--- | :--- | :--- | :--- |
| **Zone Alarm** | **Red Wall Light** | EV crosses the yellow floor line. | Doors **CLOSE**. |
| **Active Block** | **Patrol Pursuit** | Robot confirms Green Chassis + Black Wheels. | Robot **CHASES**. |
| **Pillar Rejection**| **Pursuit Abort** | Robot chases, sees no wheels (Pillar), and stops. | Robot **RESUMES PATROL**. |
| **Access Grant** | **Doors Open** | Red Non-EV is in the lobby. | Doors **OPEN**. |

---

## Hypothesis Testing & Data Logging

To rigorously evaluate **Hypothesis H1** (that feature validation reduces false positives without ruining reaction time), the system automatically generates detailed CSV logs. This allows us to empirically measure the "intelligence" of the robot.

### 1. `patrol_data.csv` (Reaction Metrics)
Logs the timestamp of every state change and alarm trigger.
*   **Key Metric:** `Duration` — We use this to measure how long the robot maintained a lock before aborting (for pillars) or blocking (for cars).

### 2. `detection_analysis.csv` (Algorithm Internals)
Logs the frame-by-frame vision data. We track specific metrics to verify the "Pillar Rejection" logic:

| Column | Description |
| :--- | :--- |
| `GreenPx` | Number of green pixels (Chassis). |
| `BlackPx` | Number of dark pixels (Wheels). |
| `ZeroCount` | **Critical:** Tracks consecutive frames with *zero* black pixels. If this hits the threshold (8), the robot aborts. |
| `PillarCheck` | Internal flag indicating if the logic suspects a pillar. |
| `Result` | Final Decision: `VEHICLE` vs `REJECTED`. |

**Verification:**
After a simulation run, checking `detection_analysis.csv` allows us to verify the exact moment the robot decided an object was a "Pillar" and released the lock.

---

## Running the Simulation

1.  Open `-Vision-Based-Monitoring-Robot/worlds/elevator_lobby.wbt` in **Webots**.
2.  **Build** both C controllers (`epuck_fsm_controller` and `epuck_camera_stationary`).
3.  Press **Play** (▶).
4.  Use the keyboard controls (click inside the 3D view first):

| Key | Function |
| :--- | :--- |
| **↑ ↓ ← →** | Drive the **Green EV** (Triggers Alarms & Pursuit). |
| **W / A / S / D** | Move the **(Human) Non-EV** (Triggers Door Open). |
| **R** | **Reset** all vehicles to start. |
| **Q** | **Quit** and save all CSV data. |

---

## Team Contribution

We divided the project workload based on our technical strengths to ensure smooth integration between the environment and the robot intelligence:

| Member | Focus Area | Key Implementation |
| :--- | :--- | :--- |
| **Yousuf H** | **Environment & Supervisor** | **Simulation Logic:**<br>Yousuf managed the `vpe_supervisor.py`, handling the complex door logic and zone alarms. He also established the environment landmarks and calibrated the stationary camera unit to ensure consistent lighting conditions. |
| **Nasrudin A** | **Perception & Control** | **Advanced Control & Vision:**<br>Nasrudin implemented the **Spatial Context Validation** and **Dynamic Revalidation** algorithms in C (`epuck_fsm_controller.c`). He also designed the **Braitenberg** obstacle avoidance matrix and the CSV logging system for evaluation. |
