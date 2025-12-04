from controller import Supervisor, Keyboard
import struct
import csv
import os
import math
from datetime import datetime
import sys

# Constants Configuration
TIME_STEP = 32
COMM_CHANNEL = 1
RESTRICTED_Y_LIMIT = -1.5 # Y-coordinate boundary for the danger zone
HYSTERESIS = 0.1         # Buffer distance to prevent rapid state switching

EV_MOVE_SPEED = 0.08

# Edge case testing positions (Y-coordinates)
EDGE_CASE_DISTANCES = [3.0, 2.5, 2.0, 1.5, 1.0, 0.5]


class VPEExperiment:
    """Manages the H1 experiment, acting as the primary zone violation detector and logger."""

    def __init__(self):
        self.supervisor = Supervisor()
        self.keyboard = self.supervisor.getKeyboard()
        self.keyboard.enable(TIME_STEP)

        self.experiment_id = datetime.now().strftime("%Y%m%d_%H%M%S")

        # this getsdevice nodes from the Webots world
        self.signal_emitter = self.supervisor.getDevice("emitter")
        self.ev_vehicle = self.supervisor.getFromDef("ElectricVehicle")
        self.patrol_robot = self.supervisor.getFromDef("PatrolRobot")
        self.wall_alarm_light = self.supervisor.getFromDef("WallAlarmLight")

        if self.signal_emitter:
            self.signal_emitter.setChannel(COMM_CHANNEL)

        # ev's translation field
        if self.ev_vehicle:
            self.ev_translation_field = self.ev_vehicle.getField("translation")
        else:
            print("[ERROR] ElectricVehicle node not found! Cannot run experiment.")
            self.ev_translation_field = None
            self.supervisor.simulationQuit(1)
            sys.exit(1)

        # variables to control the wall light's appearance
        self.point_light = None
        self.emissive_color_field = None
        self.base_color_field = None
        
        if self.wall_alarm_light:
            self._setup_alarm_light_fields()

        # experiment state tracking
        self.is_in_restricted_zone = False # True when EV is in danger zone (considering hysteresis)
        self.alarm_count = 0
        self.alarm_entry_time = 0.0
        self.total_time_in_zone = 0.0

        # data collection lists
        self.main_experiment_records = []
        self.current_alarm_details = {}
        self.edge_case_records = []

        # flashing alarm logic
        self.is_flashing_on = False
        self.flash_step_counter = 0
        self.flash_interval_steps = 8

        # Create output directory
        if not os.path.exists("experiment_data"):
            os.makedirs("experiment_data")

        self._print_user_instructions()

    def _setup_alarm_light_fields(self):
        """Looks inside the alarm light node to find the fields for control."""
        children_field = self.wall_alarm_light.getField("children")
        if not children_field:
            return

        for i in range(children_field.getCount()):
            node = children_field.getMFNode(i)
            if not node:
                continue

            node_type = node.getTypeName()
            
            if node_type == "PointLight":
                self.point_light = node
            
            elif node_type == "Pose":
                pose_children = node.getField("children")
                if pose_children and pose_children.getCount() > 0:
                    shape = pose_children.getMFNode(0)
                    if shape and shape.getTypeName() == "Shape":
                        appearance = shape.getField("appearance")
                        if appearance:
                            pbr = appearance.getSFNode()
                            if pbr:
                                self.emissive_color_field = pbr.getField("emissiveColor")
                                self.base_color_field = pbr.getField("baseColor")

    def _set_alarm_light(self, should_be_on):
        """Sets the alarm light's visual and lighting properties."""
        
        if self.point_light:
            intensity_field = self.point_light.getField("intensity")
            if intensity_field:
                intensity_field.setSFFloat(8.0 if should_be_on else 0.0)

        if self.emissive_color_field:
            self.emissive_color_field.setSFColor([1.0, 0.0, 0.0] if should_be_on else [0.0, 0.0, 0.0])

        if self.base_color_field:
            self.base_color_field.setSFColor([1.0, 0.2, 0.2] if should_be_on else [0.3, 0.0, 0.0])

    def _flash_alarm(self):
        """Toggles the alarm light on/off based on a counter to create a flashing effect."""
        self.flash_step_counter += 1
        
        if self.flash_step_counter >= self.flash_interval_steps:
            self.flash_step_counter = 0
            self.is_flashing_on = not self.is_flashing_on
            self._set_alarm_light(self.is_flashing_on)

    def _turn_alarm_off_completely(self):
        """Resets the flashing logic and ensures the light is visually off."""
        self.is_flashing_on = False
        self.flash_step_counter = 0
        self._set_alarm_light(False)

    def _print_user_instructions(self):
        """Prints the necessary controls and experiment context."""
        print("\n" + "=" * 50)
        print("VPE Supervisor – AUTHORITATIVE ALARM SOURCE")
        print(f"ID: {self.experiment_id}")
        print("=" * 50)
        print(f"Danger zone starts: Y < {RESTRICTED_Y_LIMIT:.2f}m")
        print(f"Hysteresis buffer: {HYSTERESIS}m")
        print("\nControls:")
        print("  UP/DOWN/LEFT/RIGHT - Drive the EV")
        print("  R - Reset EV to start (0, 3.0)")
        print("  S - Save all experiment data (CSV & Summary)")
        print("  T - Run a quick alarm light test")
        print("  E - Run systematic edge case tests")
        print("  D - Display current positions & distances")
        print("  Q - Quit simulation and save data")
        print("\nNOTE: VPE supervisor logic is the *only* source for zone alarms.")
        print("=" * 50 + "\n")

    def _move_ev(self, delta_x, delta_y):
        """Adjusts the EV's position by the specified delta amounts, respecting world boundaries."""
        if not self.ev_translation_field:
            return
            
        current_pos = self.ev_translation_field.getSFVec3f()
        
        # Calculate new position, clipping to the safe X/Y ranges
        new_x = max(-4.5, min(4.5, current_pos[0] + delta_x))
        new_y = max(-3.5, min(3.5, current_pos[1] + delta_y))
        
        new_pos = [new_x, new_y, current_pos[2]] 
        self.ev_translation_field.setSFVec3f(new_pos)

    def _set_ev_position(self, x, y):
        """Instantly moves the EV to a specific (X, Y) coordinate."""
        if self.ev_translation_field:
            current_pos = self.ev_translation_field.getSFVec3f()
            self.ev_translation_field.setSFVec3f([x, y, current_pos[2]])

    def _reset_ev(self):
        """Puts the EV back to its starting spot, clears the alarm."""
        if self.ev_translation_field:
            self.ev_translation_field.setSFVec3f([0.0, 3.0, 0.15]) 
        
        self._turn_alarm_off_completely()
        self.is_in_restricted_zone = False
        print("[INFO] EV reset to starting position (0.0, 3.0).")

    def _get_epuck_status(self):
        """Checks the patrol robot's velocity to see if it's moving or stopped."""
        if not self.patrol_robot:
            return "UNKNOWN"
            
        velocity = self.patrol_robot.getVelocity()
        speed = math.sqrt(velocity[0]**2 + velocity[1]**2) 
        
        return "STOPPED" if speed < 0.01 else "MOVING"

    def _calculate_ev_to_epuck_distance(self):
        """Calculates the straight-line distance between the EV and the e-puck (2D)."""
        if not self.ev_vehicle or not self.patrol_robot:
            return -1.0
            
        ev_pos = self.ev_vehicle.getPosition()
        epuck_pos = self.patrol_robot.getPosition()
        
        distance = math.sqrt(
            (ev_pos[0] - epuck_pos[0])**2 + 
            (ev_pos[1] - epuck_pos[1])**2
        )
        return distance

    def _print_positions(self):
        """Prints the current coordinates of both robots and the distance between them."""
        print("\n" + "-" * 30)
        
        if self.ev_vehicle:
            ev_pos = self.ev_vehicle.getPosition()
            print(f"EV Position: ({ev_pos[0]:.2f}, {ev_pos[1]:.2f}, {ev_pos[2]:.2f})")
            
        if self.patrol_robot:
            ep_pos = self.patrol_robot.getPosition()
            print(f"E-puck Position: ({ep_pos[0]:.2f}, {ep_pos[1]:.2f}, {ep_pos[2]:.2f})")
            
        distance = self._calculate_ev_to_epuck_distance()
        print(f"Distance (EV to E-puck): {distance:.2f}m")
        print(f"Danger Zone Boundary: Y < {RESTRICTED_Y_LIMIT}")
        print(f"EV currently alarming: {self.is_in_restricted_zone}")
        print("-" * 30 + "\n")

    def _run_edge_case_tests(self):
        """Systematically moves the EV to various Y-positions and records the robot's state."""
        print("\n" + "=" * 50)
        print("RUNNING EDGE CASE TESTING MODE")
        print("=" * 50)

        self.edge_case_records = [] 
        
        for i, y_pos in enumerate(EDGE_CASE_DISTANCES):
            print(f"\n--- Test {i+1}/{len(EDGE_CASE_DISTANCES)}: Y-Target={y_pos:.2f}m ---")
            
            self._set_ev_position(0.0, y_pos)
            
            for _ in range(20):
                self.supervisor.step(TIME_STEP)
            
            current_distance = self._calculate_ev_to_epuck_distance()
            epuck_state = self._get_epuck_status()
            is_truly_in_zone = y_pos < RESTRICTED_Y_LIMIT 
            
            record = {
                "test_num": i + 1,
                "ev_y_position": y_pos,
                "distance_to_epuck": current_distance,
                "epuck_state": epuck_state,
                "in_restricted_zone": is_truly_in_zone,
                "timestamp": self.supervisor.getTime()
            }
            self.edge_case_records.append(record)
            
            print(f"  Distance recorded: {current_distance:.2f}m")
            print(f"  E-puck state: {epuck_state}")
            print(f"  VPE Zone Check: {'YES' if is_truly_in_zone else 'NO'}")
            
            for _ in range(30):
                self.supervisor.step(TIME_STEP)

        print("\nEdge case testing finished!")
        print(f"Saved {len(self.edge_case_records)} test observations.")
        print("=" * 50 + "\n")
            
        self._reset_ev()

    def _test_alarm_light(self):
        """Runs a visual check of the wall alarm light."""
        print("[TEST] Flashing wall alarm light 5 times...")
        for i in range(10):
            self._set_alarm_light(i % 2 == 0) 
            self.supervisor.step(TIME_STEP * 4) 
            
        self._turn_alarm_off_completely()
        print("[TEST] Alarm light test complete.\n")

    def _save_all_data(self):
        """Saves all collected experiment data (main alarms, edge cases, and summary)."""
        timestamp = self.experiment_id

        # 1. save Main Alarm Records
        main_file = f"experiment_data/h1_vpe_data_{timestamp}.csv"
        with open(main_file, "w", newline="") as f: 
            writer = csv.writer(f)
            writer.writerow([
                "alarm_num", "trigger_time", "clear_time", "duration_sec",
                "ev_y_position", "distance_to_epuck", "epuck_state_at_trigger"
            ])
            for rec in self.main_experiment_records:
                writer.writerow([
                    rec.get("alarm_num"), rec.get("trigger_time"), 
                    rec.get("clear_time", "N/A"), rec.get("duration", "N/A"),
                    rec.get("ev_y"), rec.get("distance"), rec.get("epuck_state")
                ])
        print(f"[SAVED] {len(self.main_experiment_records)} alarm records -> {main_file}")

        # 2. Save Edge Case Records
        if self.edge_case_records:
            edge_file = f"experiment_data/h1_edge_cases_{timestamp}.csv"
            with open(edge_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "test_num", "ev_y_position", "distance_to_epuck",
                    "epuck_state", "in_restricted_zone", "timestamp"
                ])
                for rec in self.edge_case_records:
                    writer.writerow([
                        rec.get("test_num"), rec.get("ev_y_position"), 
                        rec.get("distance_to_epuck"), rec.get("epuck_state"), 
                        rec.get("in_restricted_zone"), rec.get("timestamp")
                    ])
            print(f"[SAVED] {len(self.edge_case_records)} edge cases -> {edge_file}")

        # 3. Save Summary Statistics
        summary_file = f"experiment_data/h1_summary_{timestamp}.txt"
        with open(summary_file, "w") as f:
            f.write("H1 Experiment Summary\n")
            f.write("=" * 40 + "\n\n")
            f.write(f"Experiment ID: {timestamp}\n")
            f.write(f"Total VPE alarms triggered: {self.alarm_count}\n")
            f.write(f"Total cumulative time in restricted zone: {self.total_time_in_zone:.2f} seconds\n")
            f.write(f"Edge case tests performed: {len(self.edge_case_records)}\n")
        print(f"[SAVED] Experiment Summary -> {summary_file}")

    def _handle_keyboard_input(self):
        """Processes keyboard commands. Returns False if the user presses 'Q'."""
        key = self.keyboard.getKey()
        
        while key != -1: 
            if key == Keyboard.UP:
                self._move_ev(0, -EV_MOVE_SPEED) 
            elif key == Keyboard.DOWN:
                self._move_ev(0, EV_MOVE_SPEED) 
            elif key == Keyboard.LEFT:
                self._move_ev(-EV_MOVE_SPEED, 0)
            elif key == Keyboard.RIGHT:
                self._move_ev(EV_MOVE_SPEED, 0)
            elif key == ord('R'):
                self._reset_ev()
            elif key == ord('S'):
                self._save_all_data()
            elif key == ord('T'):
                self._test_alarm_light()
            elif key == ord('E'):
                self._run_edge_case_tests()
            elif key == ord('D'):
                self._print_positions()
            elif key == ord('Q'):
                self._save_all_data()
                return False 
            
            key = self.keyboard.getKey()
            
        return True

    def run(self):
        """The main execution loop for the VPE Supervisor experiment."""
        
        if not self.ev_translation_field:
            return 

        print("\n[SYSTEM] VPE Monitoring has started.")

        while self.supervisor.step(TIME_STEP) != -1:
            
            if not self._handle_keyboard_input():
                break

            current_time = self.supervisor.getTime()
            
            ev_y = self.ev_vehicle.getPosition()[1]

            # Apply hysteresis logic for zone detection
            if self.is_in_restricted_zone:
                # Require crossing the hysteresis boundary to clear the alarm
                is_currently_in_zone = ev_y < (RESTRICTED_Y_LIMIT + HYSTERESIS)
            else:
                # Use the strict boundary to trigger the alarm
                is_currently_in_zone = ev_y < RESTRICTED_Y_LIMIT

            
            # ZONE ENTRY LOGIC (ALARM TRIGGER) 
            if is_currently_in_zone and not self.is_in_restricted_zone:
                self.alarm_count += 1
                self.alarm_entry_time = current_time
                
                epuck_state = self._get_epuck_status()
                distance = self._calculate_ev_to_epuck_distance()

                self.current_alarm_details = {
                    "alarm_num": self.alarm_count,
                    "trigger_time": current_time,
                    "ev_y": ev_y,
                    "distance": distance,
                    "epuck_state": epuck_state
                }

                print(f"\n{'='*50}")
                print(f" [VPE ALARM #{self.alarm_count}] ZONE VIOLATION DETECTED!")
                print(f"  Time: {current_time:.2f}s | EV Y: {ev_y:.3f}m")
                print(f"  -> Sending authoritative alarm signal to e-puck...")
                print(f"{'='*50}\n")


            # ZONE EXIT LOGIC (ALARM CLEAR) 
            elif not is_currently_in_zone and self.is_in_restricted_zone:
                duration = current_time - self.alarm_entry_time
                self.total_time_in_zone += duration
                
                # Complete the current alarm record
                self.current_alarm_details["clear_time"] = current_time
                self.current_alarm_details["duration"] = duration
                self.main_experiment_records.append(self.current_alarm_details.copy())

                self._turn_alarm_off_completely()
                
                print(f"\n[VPE ALARM #{self.alarm_count}] CLEARED")
                print(f"  Duration: {duration:.2f}s | Total cumulative: {self.total_time_in_zone:.2f}s\n")
            
            
            self.is_in_restricted_zone = is_currently_in_zone

            #  wall alarm light
            if self.is_in_restricted_zone:
                self._flash_alarm()
            
            
            if self.signal_emitter:
                #  1 (alarming) or 0 (safe)
                msg = struct.pack('B', 1 if self.is_in_restricted_zone else 0) 
                self.signal_emitter.send(msg)

        # final save upon exit
        self._save_all_data()
        
        # final summary output
        print("\n" + "=" * 50)
        print("EXPERIMENT CONCLUDED")
        print("=" * 50)
        print(f"Total Alarms: {self.alarm_count}")
        print(f"Total Time in Zone: {self.total_time_in_zone:.2f}s")
        print("=" * 50 + "\n")


if __name__ == "__main__":
    experiment = VPEExperiment()
    experiment.run()