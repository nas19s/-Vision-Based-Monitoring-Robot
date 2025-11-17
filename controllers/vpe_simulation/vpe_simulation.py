# controllers/vpe_simulation/vpe_simulation.py
from controller import Robot

# --- Setup ---
robot = Robot()
TS = int(robot.getBasicTimeStep())

cam = robot.getDevice('camera')
cam.enable(TS)

# Emitter must exist on this robot (Supervisor or a separate VPE node) and be named "emitter"
emit = robot.getDevice('emitter')
emit.setChannel(1)

def compute_alarm_from_image(img):
    """Returns 1 if blue EV + red zone are both visible; else 0."""
    if not img:
        return 0
    h, w = len(img), len(img[0])
    red = blue = 0
    for y in range(h):
        for x in range(w):
            r, g, b = img[y][x]
            # red(restricted area)
            if r > 80 and g < 90 and b < 90:
                red += 1
            # blue(EV)
            if b > 140 and r < 90 and g < 90:
                blue += 1
    total = h * w
    red_ratio = red / total
    blue_ratio = blue / total
    return 1 if (red_ratio > 0.05 and blue_ratio > 0.01) else 0

# --- Main loop ---
while robot.step(TS) != -1:
    img = cam.getImageArray()
    alarm = compute_alarm_from_image(img)
    emit.send(bytes([alarm]))
