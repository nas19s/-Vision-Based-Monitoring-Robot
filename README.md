# Elevator Safety Monitoring Robot

A Webots simulation project where an e-puck robot monitors an elevator lobby for electric vehicles (EVs) entering a restricted zone.

## What This Does

We built a safety system that detects when an EV gets too close to elevator doors. This matters because EVs with lithium batteries can be fire hazards in enclosed elevator spaces.

The system uses two detection methods:

- **Camera detection** - spots the green EV marker visually
- **Zone detection (VPE)** - triggers when EV crosses the boundary line

When either detects a violation, the robot flashes its LEDs and a wall-mounted alarm light goes off.

## Project Structure

- **FinalProject/**
  - **worlds/**
    - `elevator_lobby.wbt` – The simulation environment
  - **controllers/**
    - **epuck_fsm_controller/**
      - `epuck_fsm_controller.c` – Robot brain (FSM + camera)
      - `h1_experiment_data.csv` – Main experiment results
    - **vpe_supervisor/**
      - `vpe_supervisor.py` – Zone monitoring + alarm light
      - **experiment_data/**
        - `h1_vpe_data_[timestamp].csv` – VPE experiment logs
  - `README.md`

## How to Run

1. Open `elevator_lobby.wbt` in Webots.
2. Make sure to Build the 'epuck_fsm_controller.c' file, press the gear icon and then it will automatically create a Makefile, then Reload/Refresh.
3. Press the **Play** button.
4. Use arrow keys to move the green EV around.
5. Watch what happens when it enters the restricted (red) zone!

### Controls
- **↑↓←→** - Move the EV
- **R** - Reset EV to start
- **T** - Test the alarm light
- **S** - Save experiment data
- **Q** - Quit and save

## The Robot States

The e-puck uses a simple finite state machine (FSM):

1. **MONITORING** - Watching for the EV (one LED on)
2. **TARGET_DETECTED** - EV spotted! (half LEDs on)
3. **ALARM_ACTIVE** - Full alert (all LEDs flashing)

## Viewing the Camera

To see what the robot sees:

1. In the scene tree (left panel), expand `DEF PatrolRobot E-puck`.
2. Find and double-click `camera`.
3. A window pops up showing the robot's view.

The camera looks for bright green pixels. When it sees enough, it triggers the alarm.

## Experiment Data

We collect data in two CSV files:

### `h1_experiment_data.csv` (in `epuck_fsm_controller` folder)
Main results from the robot controller:

| Column | Description |
|--------|-------------|
| alarm_num | Which alarm this was (1, 2, 3...) |
| trigger_time_sec | When it happened |
| response_time_ms | How fast the robot reacted |
| previous_state | What the robot was doing before |
| detection_method | CAMERA or VPE |
| green_pixels | How many green pixels triggered it |

### `h1_vpe_data_[timestamp].csv` (in `vpe_supervisor/experiment_data` folder)
Logs from the supervisor tracking EV position and zone violations:

| Column | Description |
|--------|-------------|
| alarm_num | Alarm count |
| trigger_time | When EV entered restricted zone |
| clear_time | When EV left the zone |
| duration_sec | How long the violation lasted |
| ev_y_position | EV's Y coordinate at trigger |
| epuck_state_at_trigger | Whether robot was moving or stopped |

## Our Hypothesis (H1)

> A high-priority alarm behaviour can reliably suppress lower-priority behaviours and respond within one control cycle (32ms).

**Result:** Confirmed. Looking at `h1_experiment_data.csv`, response times consistently hit 32ms (one timestep). The alarm behaviour successfully overrides monitoring state every time.

## Team

- **Nasrudin A** 
- **Yousuf H**
- **Tairui Z** 


## Built With

- Webots R2023b
- C (robot controller)
- Python (supervisor)

---


