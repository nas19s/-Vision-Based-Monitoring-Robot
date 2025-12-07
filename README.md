# Elevator Safety & EV Access Control System

This Webots project simulates an autonomous safety monitoring system for an elevator lobby. We designed this to address the rising fire safety risks associated with lithium-ion batteries in enclosed spaces. The goal is simple on paper but technically complex: **allow humans to use the elevator, but actively block and lock out Electric Vehicles (Green EVs).**

Rather than just passively observing, our system implements **active defence behaviours**. We moved beyond simple obstacle avoidance to create a multi-agent system that can visually identify a threat, coordinate an alarm, and physically intercept the target.

---

## Project Overview: Intelligent Active Defence

Our solution relies on two robots working in tandem with a central building supervisor.

The core innovation in this iteration is the shift from simple “Colour Thresholding” to **Feature-Based Recognition**. We realised that relying solely on colour was too prone to errors (e.g., detecting a green recycling bin as a car). To fix this, we implemented a **Spatial Context Algorithm** that validates the *structure* of an object before acting.

1. **Smart Access Control:**
   * **Human/Non-EV:** Authorised. The elevator doors open automatically.
   * **Electric Vehicle:** Restricted. When confirmed, doors lock and the robot intercepts.
   * **False Positives (e.g., Green Boxes):** **IGNORED.** The robot is smart enough to see that a green box has no wheels, so it ignores it.

2. **Layered Surveillance:**
   * **The Patrol Robot:** The primary interceptor. It uses geometric validation to confirm a target before switching to “Pursuit Mode.”
   * **The Stationary Guard:** A fixed camera providing a second angle, ensuring redundancy.

---

## Technical Architecture

We implemented the system using three distinct controllers:

### 1. The Patrol Robot (`epuck_fsm_controller.c`)

This is the most advanced agent. We built a **Finite State Machine (FSM)** that prioritises visual intelligence over basic navigation.

* **The Vision Algorithm (Spatial Context Validation):**  
  Instead of just reacting to “Green Pixels,” the robot runs a multi-pass check to determine if an object is actually a vehicle:

  1. **Chassis Detection:** Finds the centre of mass of green pixels.  
  2. **Wheel Search:** Scans the area *specifically below* the green mass for dark, high-density pixel clusters.  
  3. **Geometric Logic:** Calculates whether the wheels are in the correct position relative to the chassis (i.e., below and to the sides).

  *Why this was necessary:* This effectively filters out false positives such as green walls or boxes.

* **Control Logic:**
  * `STATE_VALIDATING`: When potential green is seen, the robot slows down to gather 10 frames of data to confirm the structure.
  * `STATE_PURSUIT`: Only triggers if the “Vehicle Confidence” score exceeds 60%. It uses a **Proportional Controller** to smooth the intercept trajectory.
  * `STATE_ALARM_BLOCK`: Physical blocking behaviour upon interception.

### 2. The Stationary Guard (`epuck_camera_stationary.c`)

Acting as the “CCTV” of the system, this robot focuses on the elevator entrance.

* **Visual Processing:** It acts as a tripwire, analysing the scene for the specific EV colour signature.
* **Communication:** Upon detection, it broadcasts a radio signal (Channel 2) to the Supervisor. This triggers the **Blue Wall Alarm**, providing a secondary safety layer independent of the patrol robot.

### 3. The Supervisor (`vpe_supervisor.py`)

This script acts as the “Building Management System.”

* **Door Logic:** It handles the mechanics of the centre-opening elevator doors (Open for Red, Lock for Green).
* **Zone Alarm:** It monitors absolute GPS coordinates. If an EV crosses the “Safety Line” on the floor, the Red Wall Light triggers immediately.

---

## Visual & Logic Guide

We designed the visual feedback to be intuitive during the simulation:

| Component | Visual Indicator | Condition | Result |
| :--- | :--- | :--- | :--- |
| **Zone Alarm** | **Red Wall Light** | EV crosses the yellow floor line. | Doors **CLOSE**. |
| **Active Block** | **Patrol Pursuit** | Robot confirms “Green Chassis + Black Wheels”. | Robot **CHASES**. |
| **False Positive** | **Patrol Ignores** | Robot sees a Green Box (No wheels). | Robot **IGNORES**. |
| **Access Grant** | **Doors Open** | Red Non-EV is in the lobby. | Doors **OPEN**. |

---

## Hypothesis Testing & Data Logging

To properly evaluate our main hypothesis (**H1**) — *that adding feature validation reduces false positives without harming reaction time* — we implemented a comprehensive logging system.

The code automatically generates two CSV files in the root directory:

| Log File | What it Tracks |
| :--- | :--- |
| `patrol_data.csv` | **Reaction Speed:** Logs the exact timestamp of state changes. We use this to prove the robot can validate and switch to “Pursuit” mode in under 500 ms. |
| `detection_analysis.csv` | **Algorithm Reliability:** Logs internal vision metrics for every frame (`GreenPixels`, `WheelPixels`, `ConfidenceScore`, and `RejectionReason`). |

**How to verify it works:**  
Check `detection_analysis.csv` after a run. You will see rows where `Result = REJECTED` and `Reason = Wheels not below chassis`. This proves the robot is making intelligent decisions rather than just reacting to colour.

---

## Running the Simulation

1. Open `FinalProject/worlds/elevator_lobby.wbt` in **Webots**.  
2. **Build** both C controllers (`epuck_fsm_controller` and `epuck_camera_stationary`).  
3. Press **Play** (▶).  
4. Use the keyboard controls (click inside the 3D view first):

| Key | Function |
| :--- | :--- |
| **↑ ↓ ← →** | Drive the **Green EV** (Triggers Alarms & Pursuit). |
| **W / A / S / D** | Move the **(Human) Non-EV** (Triggers Door Open). |
| **R** | **Reset** all vehicles to start. |
| **Q** | **Quit** and save all CSV data. |

---

## Team Contribution

We divided the project according to our individual strengths, ensuring that the vision systems, control logic, and environment behaviour all integrated smoothly.

| Member | Focus Area | Key Contribution |
|--------|-------------|------------------|
| **Yousuf H** | **Environment & Supervisor** | **Simulation and System Logic:** Yousuf managed the `vpe_supervisor.py` script, including the door-control logic, zone-based safety triggers, and overall coordination of the elevator system. He also established the initial computer-vision setup and calibration for the stationary camera. |
| **Nasrudin A** | **Perception & Control** | **Advanced Feature Detection:** Nasrudin implemented the **Spatial Context Validation** algorithm in C, enabling the system to associate the green chassis with wheel structures to reduce false positives. He also built the Proportional Controller for smooth pursuit and developed the CSV logging system used for evaluation. |
