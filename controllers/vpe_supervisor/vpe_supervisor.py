"""
VPE Supervisor – H1 Experiment
Supervisor script for monitoring the EV and triggering an alarm
when the vehicle enters the restricted zone.

Includes: 
- alarm light flashing
- EV manual control
- logging alarm events to CSV
"""

from controller import Supervisor, Keyboard
import struct
import csv
import os
from datetime import datetime

TIME_STEP = 32
COMM_CHANNEL = 1
RESTRICTED_Y_LIMIT = -1.5
HYSTERESIS = 0.1


class VPEExperiment:
    def __init__(self):
        self.supervisor = Supervisor()
        self.keyboard = self.supervisor.getKeyboard()
        self.keyboard.enable(TIME_STEP)

        self.emitter = self.supervisor.getDevice("emitter")
        self.emitter.setChannel(COMM_CHANNEL)

        # Nodes in the world
        self.ev = self.supervisor.getFromDef("ElectricVehicle")
        self.epuck = self.supervisor.getFromDef("PatrolRobot")
        self.alarm_light = self.supervisor.getFromDef("WallAlarmLight")

        if self.ev:
            self.ev_trans_field = self.ev.getField("translation")
        else:
            print("[Error] ElectricVehicle node missing.")
            self.ev_trans_field = None

        # Alarm light fields (if available)
        self.point_light = None
        self.emissive_f = None
        self.base_color_f = None

        if self.alarm_light:
            self._init_alarm_light()

        # Experiment state
        self.ev_in_zone = False
        self.alarm_count = 0
        self.experiment_records = []
        self.current_alarm_info = {}

        # Flashing logic
        self.flash_state = False
        self.flash_counter = 0
        self.flash_interval = 8

        # Make folder for logs
        if not os.path.exists("experiment_data"):
            os.mkdir("experiment_data")

        self.experiment_id = datetime.now().strftime("%Y%m%d_%H%M%S")

        self._print_instructions()

    def _init_alarm_light(self):
        """Extract useful fields from the alarm light node."""
        children = self.alarm_light.getField("children")
        if not children:
            return

        for i in range(children.getCount()):
            node = children.getMFNode(i)
            if not node:
                continue

            t = node.getTypeName()
            if t == "PointLight":
                self.point_light = node

            elif t == "Pose":
                # The red sphere is inside the pose children
                pose_children = node.getField("children")
                if pose_children and pose_children.getCount() > 0:
                    shape = pose_children.getMFNode(0)
                    if shape and shape.getTypeName() == "Shape":
                        appearance = shape.getField("appearance")
                        if appearance:
                            pbr = appearance.getSFNode()
                            if pbr:
                                self.emissive_f = pbr.getField("emissiveColor")
                                self.base_color_f = pbr.getField("baseColor")

    def _set_alarm_light(self, on):
        """Turn alarm fully ON or OFF (no flashing toggle here)."""
        if self.point_light:
            intensity = self.point_light.getField("intensity")
            if intensity:
                intensity.setSFFloat(8.0 if on else 0.0)

        if self.emissive_f:
            self.emissive_f.setSFColor([1, 0, 0] if on else [0, 0, 0])

        if self.base_color_f:
            self.base_color_f.setSFColor([1, 0.2, 0.2] if on else [0.3, 0, 0])

    def _flash_alarm(self):
        """Flash the alarm light at a fixed interval."""
        self.flash_counter += 1
        if self.flash_counter >= self.flash_interval:
            self.flash_counter = 0
            self.flash_state = not self.flash_state
            self._set_alarm_light(self.flash_state)

    def _turn_alarm_off(self):
        self.flash_state = False
        self.flash_counter = 0
        self._set_alarm_light(False)

    def _print_instructions(self):
        print("\nVPE Experiment – H1 Version")
        print(f"Experiment ID: {self.experiment_id}")
        print("Restricted zone: y <", RESTRICTED_Y_LIMIT)
        print("Controls:")
        print("  Arrow keys – Move vehicle")
        print("  R – Reset EV")
        print("  S – Save data")
        print("  T – Test alarm light")
        print("  Q – Quit\n")

    def _move_ev(self, dx, dy):
        if not self.ev_trans_field:
            return
        pos = self.ev_trans_field.getSFVec3f()
        new_pos = [pos[0] + dx, pos[1] + dy, pos[2]]

        # Hard clamp to physical area
        new_pos[0] = max(-4.5, min(4.5, new_pos[0]))
        new_pos[1] = max(-3.5, min(3.5, new_pos[1]))

        self.ev_trans_field.setSFVec3f(new_pos)

    def _reset_ev(self):
        if self.ev_trans_field:
            self.ev_trans_field.setSFVec3f([0, 3, 0.15])
        self._turn_alarm_off()
        print("[Info] EV reset.")

    def _get_epuck_status(self):
        if not self.epuck:
            return "UNKNOWN"
        v = self.epuck.getVelocity()
        speed = (v[0]**2 + v[1]**2) ** 0.5
        return "STOPPED" if speed < 0.01 else "MOVING"

    def _save(self):
        filename = f"experiment_data/h1_vpe_data_{self.experiment_id}.csv"
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["alarm_num", "trigger_time", "clear_time",
                             "duration_sec", "ev_y_position", "epuck_state"])

            for rec in self.experiment_records:
                writer.writerow([
                    rec.get("alarm_num"),
                    rec.get("trigger_time"),
                    rec.get("clear_time"),
                    rec.get("duration"),
                    rec.get("ev_y"),
                    rec.get("epuck_state"),
                ])

        print(f"[Saved] {len(self.experiment_records)} records → {filename}")

    def _test_alarm(self):
        print("[Test] Flashing alarm briefly...")
        for i in range(10):
            self._set_alarm_light(i % 2 == 0)
            self.supervisor.step(TIME_STEP * 4)
        self._turn_alarm_off()
        print("[Test] Done.\n")

    def _handle_keyboard(self):
        key = self.keyboard.getKey()
        while key != -1:
            if key == Keyboard.UP:
                self._move_ev(0, -0.05)
            elif key == Keyboard.DOWN:
                self._move_ev(0, 0.05)
            elif key == Keyboard.LEFT:
                self._move_ev(-0.05, 0)
            elif key == Keyboard.RIGHT:
                self._move_ev(0.05, 0)
            elif key == ord('R'):
                self._reset_ev()
            elif key == ord('S'):
                self._save()
            elif key == ord('T'):
                self._test_alarm()
            elif key == ord('Q'):
                self._save()
                return False

            key = self.keyboard.getKey()

        return True

    def run(self):
        if not self.ev_trans_field:
            print("EV not available, cannot run experiment.")
            return

        print("[System] Monitoring started.\n")

        while self.supervisor.step(TIME_STEP) != -1:
            if not self._handle_keyboard():
                break

            t = self.supervisor.getTime()
            y = self.ev.getPosition()[1]

            # Apply hysteresis
            if self.ev_in_zone:
                in_zone = y < (RESTRICTED_Y_LIMIT + HYSTERESIS)
            else:
                in_zone = y < RESTRICTED_Y_LIMIT

            # Entering zone
            if in_zone and not self.ev_in_zone:
                self.alarm_count += 1
                ep_state = self._get_epuck_status()

                self.current_alarm_info = {
                    "alarm_num": self.alarm_count,
                    "trigger_time": t,
                    "ev_y": y,
                    "epuck_state": ep_state
                }

                print(f"[Alarm] Triggered #{self.alarm_count} at t={t:.2f}s (y={y:.2f})")

            # Leaving zone
            elif not in_zone and self.ev_in_zone:
                duration = t - self.current_alarm_info.get("trigger_time", t)
                self.current_alarm_info["clear_time"] = t
                self.current_alarm_info["duration"] = duration
                self.experiment_records.append(self.current_alarm_info.copy())

                self._turn_alarm_off()
                print(f"[Alarm] Cleared #{self.alarm_count} (duration {duration:.2f}s)")

            self.ev_in_zone = in_zone

            # Flash alarm if active
            if in_zone:
                self._flash_alarm()

            # Send zone status to e-puck
            msg = struct.pack('B', 1 if in_zone else 0)
            self.emitter.send(msg)

        # Save once more on exit
        self._save()
        print("[System] Experiment ended.")


if __name__ == "__main__":
    VPEExperiment().run()
