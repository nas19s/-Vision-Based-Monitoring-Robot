from controller import Supervisor, Keyboard
import struct
import csv
import os
from datetime import datetime

# --- Configuration Constants ---
TIME_STEP = 32
ZONE_CHANNEL = 1
CAMERA_CHANNEL = 2
RESTRICTED_Y_LIMIT = -1.5
HYSTERESIS = 0.1
EV_MOVE_SPEED = 0.08

# Elevator zone configuration
ELEVATOR_ZONE_Y_MIN = -3.5
ELEVATOR_ZONE_Y_MAX = -2.0
ELEVATOR_ZONE_X_MIN = -3.0
ELEVATOR_ZONE_X_MAX = 3.0

# Door control - CENTER OPENING
DOOR_OPEN_POSITION = 0.5   # How far each door slides
DOOR_VELOCITY = 0.3


class VPEExperiment:
    """Dual alarm system with center-opening elevator doors."""

    def __init__(self):
        self.supervisor = Supervisor()
        self.keyboard = self.supervisor.getKeyboard()
        self.keyboard.enable(TIME_STEP)

        self.experiment_id = datetime.now().strftime("%Y%m%d_%H%M%S")

        # --- Devices ---
        self.signal_emitter = self.supervisor.getDevice("emitter")
        if self.signal_emitter:
            self.signal_emitter.setChannel(ZONE_CHANNEL)
            print(f"[INIT] Emitter: OK (channel {ZONE_CHANNEL})")

        self.cam2_receiver = self.supervisor.getDevice("receiver")
        if self.cam2_receiver:
            self.cam2_receiver.enable(TIME_STEP)
            self.cam2_receiver.setChannel(CAMERA_CHANNEL)
            print(f"[INIT] Receiver: OK (channel {CAMERA_CHANNEL})")

        # --- World nodes ---
        self.ev_vehicle = self.supervisor.getFromDef("ElectricVehicle")
        self.non_ev_vehicle = self.supervisor.getFromDef("NonEVVehicle")
        self.wall_alarm_light = self.supervisor.getFromDef("WallAlarmLight")
        self.camera_alarm_light = self.supervisor.getFromDef("CameraAlarmLight")

        # Vehicle translation fields
        self.ev_translation_field = None
        self.non_ev_translation_field = None

        if self.ev_vehicle:
            self.ev_translation_field = self.ev_vehicle.getField("translation")
            print("[INIT] ElectricVehicle (GREEN): OK")
        else:
            print("[WARNING] ElectricVehicle not found")

        if self.non_ev_vehicle:
            self.non_ev_translation_field = self.non_ev_vehicle.getField("translation")
            print("[INIT] NonEVVehicle (RED): OK")
        else:
            print("[INFO] NonEVVehicle not found - add DEF NonEVVehicle to world")

        # --- Elevator Door Control ---
        # Find door robots and their joint position fields
        self.door_joints = {}
        door_configs = [
            ('elevator1_door_left', 'elev1_left', 'left'),
            ('elevator1_door_right', 'elev1_right', 'right'),
            ('elevator2_door_left', 'elev2_left', 'left'),
            ('elevator2_door_right', 'elev2_right', 'right'),
        ]

        for robot_name, key, side in door_configs:
            robot = self._find_robot_by_name(robot_name)
            if robot:
                joint_field = self._get_joint_position_field(robot)
                if joint_field:
                    self.door_joints[key] = {
                        'field': joint_field,
                        'side': side,
                        'robot': robot
                    }
                    print(f"[INIT] {robot_name}: OK")
                else:
                    print(f"[INIT] {robot_name}: Joint not found")
            else:
                print(f"[INIT] {robot_name}: Robot not found")

        # --- Alarm light fields ---
        self.zone_point_light = None
        self.zone_emissive_field = None
        self.zone_base_field = None
        self.camera_point_light = None
        self.camera_emissive_field = None
        self.camera_base_field = None

        if self.wall_alarm_light:
            self._setup_light_fields(self.wall_alarm_light, "zone")
        if self.camera_alarm_light:
            self._setup_light_fields(self.camera_alarm_light, "camera")

        # --- State tracking ---
        self.is_in_restricted_zone = False
        self.camera_detected_ev = False
        self.doors_target_open = False
        self.current_door_position = 0.0

        self.alarm_count = 0
        self.camera_alarm_count = 0
        self.alarm_entry_time = 0.0
        self.camera_alarm_start_time = 0.0

        # --- Data collection ---
        self.zone_alarm_records = []
        self.camera_alarm_records = []
        self.elevator_access_records = []
        self.current_alarm_details = {}

        # --- Flash timing ---
        self.zone_flash_on = False
        self.zone_flash_counter = 0
        self.camera_flash_on = False
        self.camera_flash_counter = 0

        if not os.path.exists("experiment_data"):
            os.makedirs("experiment_data")

        self._print_instructions()

    def _find_robot_by_name(self, name):
        """Find a robot node by its name field."""
        root = self.supervisor.getRoot()
        children = root.getField("children")
        for i in range(children.getCount()):
            node = children.getMFNode(i)
            if node:
                name_field = node.getField("name")
                if name_field and name_field.getSFString() == name:
                    return node
        return None

    def _get_joint_position_field(self, robot_node):
        """Get the position field from a robot's SliderJoint."""
        children = robot_node.getField("children")
        if not children or children.getCount() == 0:
            return None

        slider_joint = children.getMFNode(0)
        if not slider_joint or slider_joint.getTypeName() != "SliderJoint":
            return None

        joint_params = slider_joint.getField("jointParameters")
        if not joint_params:
            return None

        params_node = joint_params.getSFNode()
        if not params_node:
            return None

        return params_node.getField("position")

    def _setup_light_fields(self, light_node, light_type):
        """Extract controllable fields from alarm light node."""
        children = light_node.getField("children")
        if not children:
            return

        for i in range(children.getCount()):
            node = children.getMFNode(i)
            if not node:
                continue

            if node.getTypeName() == "PointLight":
                if light_type == "zone":
                    self.zone_point_light = node
                else:
                    self.camera_point_light = node

            elif node.getTypeName() == "Pose":
                pose_children = node.getField("children")
                if pose_children and pose_children.getCount() > 0:
                    shape = pose_children.getMFNode(0)
                    if shape and shape.getTypeName() == "Shape":
                        appearance = shape.getField("appearance")
                        if appearance:
                            pbr = appearance.getSFNode()
                            if pbr:
                                if light_type == "zone":
                                    self.zone_emissive_field = pbr.getField("emissiveColor")
                                    self.zone_base_field = pbr.getField("baseColor")
                                else:
                                    self.camera_emissive_field = pbr.getField("emissiveColor")
                                    self.camera_base_field = pbr.getField("baseColor")

        print(f"[INIT] {'Zone' if light_type == 'zone' else 'Camera'} alarm light: OK")

    # --- Light Control ---

    def _set_zone_light(self, on):
        if self.zone_point_light:
            intensity = self.zone_point_light.getField("intensity")
            if intensity:
                intensity.setSFFloat(8.0 if on else 0.0)
        if self.zone_emissive_field:
            self.zone_emissive_field.setSFColor([1.0, 0.0, 0.0] if on else [0.0, 0.0, 0.0])
        if self.zone_base_field:
            self.zone_base_field.setSFColor([1.0, 0.2, 0.2] if on else [0.3, 0.0, 0.0])

    def _set_camera_light(self, on):
        if self.camera_point_light:
            intensity = self.camera_point_light.getField("intensity")
            if intensity:
                intensity.setSFFloat(10.0 if on else 0.0)
        if self.camera_emissive_field:
            self.camera_emissive_field.setSFColor([0.0, 0.5, 1.0] if on else [0.0, 0.0, 0.0])
        if self.camera_base_field:
            self.camera_base_field.setSFColor([0.2, 0.6, 1.0] if on else [0.0, 0.0, 0.3])

    def _flash_zone_alarm(self):
        self.zone_flash_counter += 1
        if self.zone_flash_counter >= 8:
            self.zone_flash_counter = 0
            self.zone_flash_on = not self.zone_flash_on
            self._set_zone_light(self.zone_flash_on)

    def _flash_camera_alarm(self):
        self.camera_flash_counter += 1
        if self.camera_flash_counter >= 4:
            self.camera_flash_counter = 0
            self.camera_flash_on = not self.camera_flash_on
            self._set_camera_light(self.camera_flash_on)

    def _turn_zone_light_off(self):
        self.zone_flash_on = False
        self.zone_flash_counter = 0
        self._set_zone_light(False)

    def _turn_camera_light_off(self):
        self.camera_flash_on = False
        self.camera_flash_counter = 0
        self._set_camera_light(False)

    # --- CENTER-OPENING Elevator Door Control ---

    def _update_elevator_doors(self):
        """Smoothly animate center-opening elevator doors."""
        target = DOOR_OPEN_POSITION if self.doors_target_open else 0.0
        speed = 0.015  # Position change per timestep

        if abs(self.current_door_position - target) < speed:
            self.current_door_position = target
        elif self.current_door_position < target:
            self.current_door_position += speed
        else:
            self.current_door_position -= speed

        for key, door_data in self.door_joints.items():
            field = door_data['field']
            side = door_data['side']

            if field:
                if side == 'left':
                    field.setSFFloat(-self.current_door_position)
                else:
                    field.setSFFloat(self.current_door_position)

    def _is_in_elevator_zone(self, x, y):
        """Check if position is in elevator zone."""
        return (ELEVATOR_ZONE_X_MIN <= x <= ELEVATOR_ZONE_X_MAX and
                ELEVATOR_ZONE_Y_MIN <= y <= ELEVATOR_ZONE_Y_MAX)

    # --- Utilities ---

    def _print_instructions(self):
        print("\n" + "=" * 65)
        print("   VPE SUPERVISOR - EV DETECTION & ELEVATOR CONTROL")
        print("=" * 65)
        print("\nSystem logic:")
        print("   - Green (EV) vehicle: zone and camera alarms active, elevator doors closed.")
        print("   - Red (non-EV) vehicle: no alarms, elevator doors can open.")
        print("   - No vehicle in elevator zone: doors closed.")
        print("\nControls:")
        print("   Arrow Keys     Move GREEN EV")
        print("   W/A/S/D        Move RED Non-EV")
        print("   R              Reset vehicles")
        print("   T              Test alarm lights and doors")
        print("   E              Manual door toggle")
        print("   P              Print positions")
        print("   Q              Quit and save data")
        print("=" * 65 + "\n")

    def _move_ev(self, dx, dy):
        if not self.ev_translation_field:
            return
        pos = self.ev_translation_field.getSFVec3f()
        new_x = max(-4.5, min(4.5, pos[0] + dx))
        new_y = max(-3.5, min(3.5, pos[1] + dy))
        self.ev_translation_field.setSFVec3f([new_x, new_y, pos[2]])

    def _move_non_ev(self, dx, dy):
        if not self.non_ev_translation_field:
            return
        pos = self.non_ev_translation_field.getSFVec3f()
        new_x = max(-4.5, min(4.5, pos[0] + dx))
        new_y = max(-3.5, min(3.5, pos[1] + dy))
        self.non_ev_translation_field.setSFVec3f([new_x, new_y, pos[2]])

    def _reset_vehicles(self):
        if self.ev_translation_field:
            self.ev_translation_field.setSFVec3f([-0.5, 0.9, 0.124])
        if self.non_ev_translation_field:
            self.non_ev_translation_field.setSFVec3f([1.5, 0.9, 0.124])
        self._turn_zone_light_off()
        self._turn_camera_light_off()
        self.is_in_restricted_zone = False
        self.camera_detected_ev = False
        self.doors_target_open = False
        print("[RESET] All vehicles reset to starting positions\n")

    def _print_positions(self):
        print("\n" + "-" * 55)
        if self.ev_vehicle:
            pos = self.ev_vehicle.getPosition()
            in_elev = "ELEVATOR ZONE" if self._is_in_elevator_zone(pos[0], pos[1]) else ""
            in_rest = "RESTRICTED" if pos[1] < RESTRICTED_Y_LIMIT else ""
            print(f"EV (green):   X={pos[0]:6.2f}  Y={pos[1]:6.2f}  {in_elev} {in_rest}")
        if self.non_ev_vehicle:
            pos = self.non_ev_vehicle.getPosition()
            in_elev = "ELEVATOR ZONE" if self._is_in_elevator_zone(pos[0], pos[1]) else ""
            print(f"Non-EV (red): X={pos[0]:6.2f}  Y={pos[1]:6.2f}  {in_elev}")
        print()
        print(f"   Zone Alarm:    {'ACTIVE' if self.is_in_restricted_zone else 'Off'}")
        print(f"   Camera Alarm:  {'ACTIVE' if self.camera_detected_ev else 'Off'}")
        print(f"   Elevator:      {'OPEN' if self.doors_target_open else 'CLOSED'}")
        print(f"   Door Position: {self.current_door_position:.2f} / {DOOR_OPEN_POSITION:.2f}")
        print("-" * 55 + "\n")

    def _test_lights(self):
        print("[TEST] Testing alarm lights...")
        for i in range(6):
            self._set_zone_light(i % 2 == 0)
            self._set_camera_light(i % 2 == 0)
            self.supervisor.step(TIME_STEP * 5)
        self._turn_zone_light_off()
        self._turn_camera_light_off()

        print("[TEST] Testing elevator doors...")
        self.doors_target_open = True
        for _ in range(50):
            self._update_elevator_doors()
            self.supervisor.step(TIME_STEP)
        self.doors_target_open = False
        for _ in range(50):
            self._update_elevator_doors()
            self.supervisor.step(TIME_STEP)
        print("[TEST] Complete!\n")

    def _save_data(self):
        ts = self.experiment_id

        with open(f"experiment_data/zone_alarms_{ts}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["alarm_num", "trigger_time", "clear_time", "duration", "ev_y"])
            for r in self.zone_alarm_records:
                w.writerow([
                    r.get("num"),
                    r.get("start"),
                    r.get("end"),
                    r.get("duration"),
                    r.get("ev_y")
                ])

        with open(f"experiment_data/camera_alarms_{ts}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["alarm_num", "trigger_time", "clear_time", "duration"])
            for r in self.camera_alarm_records:
                w.writerow([
                    r.get("num"),
                    r.get("start"),
                    r.get("end"),
                    r.get("duration")
                ])

        with open(f"experiment_data/elevator_access_{ts}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time", "vehicle_type", "access_granted", "reason"])
            for r in self.elevator_access_records:
                w.writerow([
                    r.get("time"),
                    r.get("vehicle"),
                    r.get("granted"),
                    r.get("reason")
                ])

        print("[SAVED] Data exported to experiment_data/\n")

    def _handle_keyboard(self):
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
            elif key == ord('W'):
                self._move_non_ev(0, -EV_MOVE_SPEED)
            elif key == ord('S'):
                self._move_non_ev(0, EV_MOVE_SPEED)
            elif key == ord('A'):
                self._move_non_ev(-EV_MOVE_SPEED, 0)
            elif key == ord('D'):
                self._move_non_ev(EV_MOVE_SPEED, 0)
            elif key == ord('R'):
                self._reset_vehicles()
            elif key == ord('T'):
                self._test_lights()
            elif key == ord('E'):
                self.doors_target_open = not self.doors_target_open
                print(f"[MANUAL] Elevator doors: {'OPENING' if self.doors_target_open else 'CLOSING'}")
            elif key == ord('P'):
                self._print_positions()
            elif key == ord('Q'):
                self._save_data()
                return False
            key = self.keyboard.getKey()
        return True

    def _check_camera_detection(self):
        """Check if camera detected GREEN (EV)."""
        if not self.cam2_receiver:
            return self.camera_detected_ev

        detected = self.camera_detected_ev
        while self.cam2_receiver.getQueueLength() > 0:
            data = self.cam2_receiver.getBytes()
            if data and len(data) > 0:
                detected = (data[0] == 1)
            self.cam2_receiver.nextPacket()
        return detected

    # --- Main Loop ---

    def run(self):
        print("[SYSTEM] Monitoring started...\n")

        while self.supervisor.step(TIME_STEP) != -1:
            if not self._handle_keyboard():
                break

            current_time = self.supervisor.getTime()

            # Get vehicle positions
            ev_pos = self.ev_vehicle.getPosition() if self.ev_vehicle else None
            ev_x = ev_pos[0] if ev_pos else 0
            ev_y = ev_pos[1] if ev_pos else 999

            non_ev_pos = self.non_ev_vehicle.getPosition() if self.non_ev_vehicle else None
            non_ev_x = non_ev_pos[0] if non_ev_pos else 0
            non_ev_y = non_ev_pos[1] if non_ev_pos else 999

            # ========== ZONE ALARM (RED) - ONLY for GREEN EV ==========
            prev_in_zone = self.is_in_restricted_zone

            if self.is_in_restricted_zone:
                self.is_in_restricted_zone = ev_y < (RESTRICTED_Y_LIMIT + HYSTERESIS)
            else:
                self.is_in_restricted_zone = ev_y < RESTRICTED_Y_LIMIT

            # Zone entry
            if self.is_in_restricted_zone and not prev_in_zone:
                self.alarm_count += 1
                self.alarm_entry_time = current_time
                self.current_alarm_details = {
                    "num": self.alarm_count,
                    "start": current_time,
                    "ev_y": ev_y
                }
                print("\n" + "=" * 50)
                print(f"ZONE ALARM #{self.alarm_count} - GREEN EV IN RESTRICTED ZONE")
                print(f"Position: Y = {ev_y:.2f} (limit: {RESTRICTED_Y_LIMIT})")
                print("Elevator doors will remain CLOSED")
                print("=" * 50 + "\n")

            # Zone exit
            elif not self.is_in_restricted_zone and prev_in_zone:
                duration = current_time - self.alarm_entry_time
                self.current_alarm_details["end"] = current_time
                self.current_alarm_details["duration"] = duration
                self.zone_alarm_records.append(self.current_alarm_details.copy())
                self._turn_zone_light_off()
                print(f"Zone Alarm #{self.alarm_count} CLEARED ({duration:.1f}s)\n")

            if self.is_in_restricted_zone:
                self._flash_zone_alarm()

            # ========== CAMERA ALARM (BLUE) - ONLY for GREEN ==========
            prev_camera = self.camera_detected_ev
            self.camera_detected_ev = self._check_camera_detection()

            if self.camera_detected_ev and not prev_camera:
                self.camera_alarm_count += 1
                self.camera_alarm_start_time = current_time
                print("\n" + "=" * 50)
                print(f"CAMERA ALARM #{self.camera_alarm_count} - GREEN EV DETECTED")
                print("Camera sees green color signature.")
                print("Elevator doors will remain CLOSED")
                print("=" * 50 + "\n")

            elif not self.camera_detected_ev and prev_camera:
                duration = current_time - self.camera_alarm_start_time
                self.camera_alarm_records.append({
                    "num": self.camera_alarm_count,
                    "start": self.camera_alarm_start_time,
                    "end": current_time,
                    "duration": duration
                })
                self._turn_camera_light_off()
                print(f"Camera Alarm #{self.camera_alarm_count} CLEARED ({duration:.1f}s)\n")

            if self.camera_detected_ev:
                self._flash_camera_alarm()

            # ========== ELEVATOR DOOR LOGIC ==========
            any_alarm_active = self.is_in_restricted_zone or self.camera_detected_ev

            ev_in_elevator = self._is_in_elevator_zone(ev_x, ev_y) if ev_pos else False
            non_ev_in_elevator = self._is_in_elevator_zone(non_ev_x, non_ev_y) if non_ev_pos else False

            prev_door_target = self.doors_target_open

            if any_alarm_active:
                # GREEN EV detected - CLOSE doors
                self.doors_target_open = False
                if prev_door_target:
                    print("[ELEVATOR] Doors CLOSING - alarm active (GREEN EV detected)")
                    self.elevator_access_records.append({
                        "time": current_time,
                        "vehicle": "ev",
                        "granted": False,
                        "reason": "alarm_active"
                    })
            elif non_ev_in_elevator:
                # Non-EV in zone, no alarm - OPEN doors
                self.doors_target_open = True
                if not prev_door_target:
                    print("[ELEVATOR] Doors OPENING - Non-EV vehicle access granted")
                    self.elevator_access_records.append({
                        "time": current_time,
                        "vehicle": "non_ev",
                        "granted": True,
                        "reason": "no_alarm"
                    })
            else:
                # No vehicle in zone
                self.doors_target_open = False
                if prev_door_target:
                    print("[ELEVATOR] Doors CLOSING - no vehicle in zone")

            # Update door animation
            self._update_elevator_doors()

            # ========== SEND STATUS TO ROBOTS ==========
            if self.signal_emitter:
                msg = struct.pack('B', 1 if self.is_in_restricted_zone else 0)
                self.signal_emitter.send(msg)

            # ========== PERIODIC STATUS ==========
            if int(current_time * 10) % 150 == 0:
                zone_st = "ON" if self.is_in_restricted_zone else "OFF"
                cam_st = "ON" if self.camera_detected_ev else "OFF"
                door_st = "OPEN" if self.doors_target_open else "CLOSED"
                door_pct = int((self.current_door_position / DOOR_OPEN_POSITION) * 100)
                print(
                    f"[{current_time:6.1f}s] "
                    f"Zone:{zone_st} Cam:{cam_st} Door:{door_st}({door_pct}%) | "
                    f"EV:({ev_x:5.1f},{ev_y:5.1f}) NonEV:({non_ev_x:5.1f},{non_ev_y:5.1f})"
                )

        self._save_data()
        print("\n" + "=" * 55)
        print("   EXPERIMENT COMPLETE")
        print("=" * 55)
        print(f"   Zone Alarms:     {self.alarm_count}")
        print(f"   Camera Alarms:   {self.camera_alarm_count}")
        print(f"   Elevator Events: {len(self.elevator_access_records)}")
        print("=" * 55 + "\n")


if __name__ == "__main__":
    experiment = VPEExperiment()
    experiment.run()
