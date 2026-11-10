"""TRIDENT X boat: 4 thrusters, gripper servo and rear flipper servo.

MATCH CONTROLS
    Left joystick  UP / DOWN     -> straight FORWARD / BACKWARD (centre = stop)   stick FOUR, flipped
    Right joystick LEFT / RIGHT  -> turn LEFT / RIGHT (centre = no turn)          stick ONE, flipped
        both at once = forward or backward while turning
        (differential thrust: turn right = left side more forward, right side more backward)
    Hold Y / hold B              -> gripper open / close (stays where you let go)
    Tap X                        -> flipper down / up
    Tap A                        -> slow mode on/off (60 % power)

SETUP ONLY (not needed in a match)
    D-pad LEFT / RIGHT / DOWN    -> balance: turn the left / right side down 2 %, or reset
    + hold RIGHT_TRIGGER         -> same, but for driving backward
    Hold RIGHT_TRIGGER + tap UP  -> thruster test: each thruster alone at 20 %, FORWARD command,
                                    water must go BACKWARD (press again to stop)
    Driver screen, once a second -> raw stick numbers "sticks 1=.. 2=.. 3=.. 4=.." to check the sticks
"""

import json
import time
from pathlib import Path
from marlin import Robot, Thruster, Servo, IMU, RadioBeacon

import auton

# ---------------- ports ----------------
# FINAL wiring (shortest routes): left parts on the M1-M4 row, right parts on the M5-M7 row.
IMU_PORT = "S6"
RADIOBEACON_PORT = "S1"
GRIPPER = "M4"   # gripper servo, at the FRONT (bow) of the boat (None = not fitted)
FLIPPER = "M2"   # flipper servo at the BACK (None = not plugged in)

# Thrusters: front = gripper end (the bow), back = flipper end.
# If the thruster test shows a "LEFT" thruster is really on the right, swap the ports below.
LEFT_THRUSTERS = ("M3", "M1")    # (front-left, back-left)
RIGHT_THRUSTERS = ("M5", "M7")   # (front-right, back-right)
ALL_THRUSTERS = LEFT_THRUSTERS + RIGHT_THRUSTERS

# Spin direction of each thruster for a FORWARD command. True = that thruster's power is flipped.
# This covers how each thruster is mounted (front one faces front, back one faces back) and its
# CW/CCW propeller. Do NOT guess: run the thruster test and, for every thruster whose water goes
# FORWARD (toward the gripper), flip its True/False. Then all four push the boat forward.
# Current values were meant to give LF anticlockwise, LB clockwise, RF anticlockwise, RB clockwise.
REVERSED = {"M3": False, "M1": True,    # left front ANTICLOCKWISE, left back CLOCKWISE
            "M5": True, "M7": False}    # right front ANTICLOCKWISE, right back CLOCKWISE

# Power of each thruster, 1.0 = full (the highest). Only turn one down (e.g. 0.9) if it is
# stronger than the others and makes the boat pull to one side.
THRUSTER_POWER = {"M3": 1.0, "M1": 1.0, "M5": 1.0, "M7": 1.0}

# ---------------- driving ----------------
# Left joystick UP/DOWN = forward/backward, right joystick LEFT/RIGHT = turn left/right.
# The Driver screen prints the raw sticks once a second ("sticks 1=.. 2=.. 3=.. 4=..").
# Check them before driving:
#   left joystick UP     -> the THROTTLE_STICK number x THROTTLE_SIGN must come out POSITIVE
#   right joystick RIGHT -> the STEER_STICK number x STEER_SIGN must come out POSITIVE
#   if the wrong number moves, put the number that DOES move in THROTTLE_STICK / STEER_STICK.
# Our controller sends its sticks rotated 180 deg from the library's labels (measured 2026-11-11:
# right joystick up/down = stick TWO and left joystick left/right = stick THREE, both reversed).
# So the left joystick's up/down is stick FOUR and the right joystick's left/right is stick ONE,
# both reversed too. (Previous layout, tested OK: THROTTLE "TWO", STEER "THREE", both -1.)
THROTTLE_STICK = "FOUR"    # left joystick, up/down
STEER_STICK = "ONE"        # right joystick, left/right
THROTTLE_SIGN = -1         # left joystick UP reads negative -> flip so UP = forward
STEER_SIGN = -1            # right joystick RIGHT reads negative -> flip so RIGHT = turn right

# 1.0 = normal. 3.0 = full power with the stick only 1/3 of the way.
# NOTE: these go by the RAW stick direction (before THROTTLE_SIGN / STEER_SIGN). With the signs
# at -1, BACKWARD_GAIN is used when driving FORWARD (and for turning left) and FORWARD_GAIN when
# reversing (and turning right). Keep them equal unless you mean that.
FORWARD_GAIN = 3.0 
BACKWARD_GAIN = 3.0    # full reverse at about half stick too
DEADZONE = 15          # sticks rest up to 13 away from 0, so ignore anything below 15
SLOW_POWER = 0.6       # power in slow mode (tap A)
RAMP_STEP = 500 / 50   # smoothing: 0 to full in 0.2 s (loop runs 50 times a second).
                       # Too jerky? Use 250 / 50 (0.4 s).

# Balance between the two sides, saved to balance.json and used in Auton too.
# Negative = left side turned down, positive = right side turned down.
BALANCE_FILE = Path(__file__).resolve().parent.parent / "balance.json"
BALANCE_STEP = 0.02
BALANCE_LIMIT = 0.5

# ---------------- servos ----------------
SERVO_SPEED = 360  # deg/s, the MARLIN maximum

GRIPPER_CLOSED_DEG = 0    # auton uses ARM_CLOSED_DEG = 0 too
GRIPPER_OPEN_DEG = 180    # auton uses ARM_OPEN_DEG = 180 too
# If the gripper hums or strains at an end, bring that angle in by about 5.

FLIPPER_UP_DEG = 0        # resting position
FLIPPER_DOWN_DEG = 180    # flipped position. Adjust both if the flipper hits the hull.

# Thruster test (hold RIGHT_TRIGGER + tap UP): each thruster ALONE, low power, with the
# FORWARD command - the same command it gets when the left joystick is pushed UP.
# For every thruster the water must go BACKWARD (toward the flipper). If it goes forward,
# flip that thruster's True/False in REVERSED above. (name, ports, duty, seconds)
TEST_DUTY = 20
TEST_STEPS = []
for _name, _port in (("LEFT FRONT", "M3"), ("LEFT BACK", "M1"),
                     ("RIGHT FRONT", "M5"), ("RIGHT BACK", "M7")):
    TEST_STEPS += [(_name, (_port,), TEST_DUTY, 3), ("pause", (), 0, 2)]

# SERVO TEST: set to True, run Driver, and every servo socket below swings 0 <-> 180 on its own
# (no buttons needed). Plug the gripper servo into each socket in turn to find a working one.
# Set back to False for normal driving.
SERVO_TEST = False
SERVO_TEST_PORTS = ("M4", "M6", "M8")   # gripper socket + spares (flipper M2 left alone)

BUTTONS = ("UP", "DOWN", "LEFT", "RIGHT", "X", "Y", "A", "B", "LEFT_TRIGGER", "RIGHT_TRIGGER")

robot = Robot()

robot.configure(
    sensors={
        IMU_PORT: IMU,
        RADIOBEACON_PORT: RadioBeacon,
    },
    effectors={
        **{port: Thruster for port in ALL_THRUSTERS},
        **({GRIPPER: Servo} if GRIPPER else {}),
        **({FLIPPER: Servo} if FLIPPER else {}),
        **({port: Servo for port in SERVO_TEST_PORTS} if SERVO_TEST else {}),
    },
)


# ---------------- helpers ----------------
def set_thruster(port, duty):
    """Power one thruster: scaled by THRUSTER_POWER, flipped if it is marked REVERSED."""
    duty *= THRUSTER_POWER.get(port, 1.0)
    robot.thruster(port).set_duty(-duty if REVERSED[port] else duty)


def set_flipper(angle):
    """Move the flipper, or do nothing while FLIPPER is None (not plugged in)."""
    if FLIPPER:
        robot.servo(FLIPPER).set_speed(SERVO_SPEED)
        robot.servo(FLIPPER).angle(angle)


def drive_sides(left, right):
    """Tank drive: one power for both left thrusters, one for both right thrusters."""
    for port in LEFT_THRUSTERS:
        set_thruster(port, left)
    for port in RIGHT_THRUSTERS:
        set_thruster(port, right)


def read_stick(name):
    """-100..100: deadzone first (on the real stick position), then the forward/backward gain."""
    value = robot.controller.joystick(name).value
    if abs(value) < DEADZONE:
        return 0.0  # a resting stick (up to 13 off centre) never moves the boat
    gain = FORWARD_GAIN if value > 0 else BACKWARD_GAIN
    sign = 1 if value > 0 else -1
    return sign * min(100.0, (abs(value) - DEADZONE) / (100 - DEADZONE) * 100 * gain)


def load_balance():
    try:
        saved = json.loads(BALANCE_FILE.read_text())
        return {"forward": float(saved["forward"]), "reverse": float(saved["reverse"])}
    except (OSError, ValueError, KeyError, TypeError):
        return {"forward": 0.0, "reverse": 0.0}


def save_balance(balance):
    try:
        BALANCE_FILE.write_text(json.dumps(balance, indent=2))
    except OSError as error:
        print("Could not save balance:", error)


def scales(value):
    """(left_scale, right_scale) for one balance value. The weaker side stays at 1.0."""
    return 1.0 - max(0.0, -value), 1.0 - max(0.0, value)


def balanced(left, right, balance):
    left_scale = scales(balance["forward" if left >= 0 else "reverse"])[0]
    right_scale = scales(balance["forward" if right >= 0 else "reverse"])[1]
    return left * left_scale, right * right_scale


def describe(value):
    left_scale, right_scale = scales(value)
    return f"left {left_scale:.0%}, right {right_scale:.0%}"


# ---------------- periods ----------------
def initialize(robot):
    time.sleep(0.5)
    print("Battery:", robot.battery_voltage, "V")
    if GRIPPER:
        robot.servo(GRIPPER).set_speed(SERVO_SPEED)
    set_flipper(FLIPPER_UP_DEG)
    auton.check_sensors(robot, IMU_PORT, RADIOBEACON_PORT)  # prints OK / NO DATA for each
    auton.print_start()  # which Auton mode is set and exactly where to put the boat
    auton.calibrate_imu(robot, IMU_PORT)  # boat is still here: measure the gyro bias


def autonomous(robot):
    # Auton steers like the sticks: left side against right side, with the saved balance.
    set_flipper(FLIPPER_UP_DEG)
    saved = load_balance()
    auton.run(
        robot,
        imu_port=IMU_PORT,
        radio_port=RADIOBEACON_PORT,
        arm_port=GRIPPER,
        drive=drive_sides,
        balance=lambda left, right: balanced(left, right, saved),
        ramp_step=RAMP_STEP,
    )


def servo_test(robot):
    """Swing every SERVO_TEST_PORTS socket between 0 and 180 deg until the period ends."""
    print("SERVO TEST: sockets", ", ".join(SERVO_TEST_PORTS), "swing 0 <-> 180 every 2 s. "
          "Move the gripper servo plug from socket to socket and watch it.")
    for port in ALL_THRUSTERS:
        set_thruster(port, 0.0)
    for port in SERVO_TEST_PORTS:
        robot.servo(port).set_speed(SERVO_SPEED)
    loop = 0
    while robot.running:
        angle = 180 if (loop // 100) % 2 else 0   # 100 loops = 2 s at 50 loops a second
        for port in SERVO_TEST_PORTS:
            robot.servo(port).angle(angle)
        if loop % 100 == 0:
            print(f"SERVO TEST: all of {', '.join(SERVO_TEST_PORTS)} -> {angle} deg")
        loop += 1


def driver(robot):
    if SERVO_TEST:
        servo_test(robot)
        return
    c = robot.controller
    balance = load_balance()
    slow = False
    flipper_down = False
    gripper = GRIPPER_CLOSED_DEG
    gripper_moving = False
    left_now = right_now = 0.0
    was_down = {name: False for name in BUTTONS}
    test_step = None
    test_loops = 0
    loop = 0

    if GRIPPER:
        robot.servo(GRIPPER).set_speed(SERVO_SPEED)
    set_flipper(FLIPPER_UP_DEG)
    print(f"DRIVER: stick {THROTTLE_STICK} = forward/back, {STEER_STICK} = turn | "
          + ("hold Y open, B close | " if GRIPPER else "") + "tap X flipper | tap A slow mode")

    while robot.running:
        # ----- buttons (act once per tap) -----
        backward_mode = c.button("RIGHT_TRIGGER").is_down
        for name in BUTTONS:
            down = c.button(name).is_down
            if down and not was_down[name]:
                if name == "X":
                    flipper_down = not flipper_down
                    print("FLIPPER:", ("DOWN" if flipper_down else "UP") if FLIPPER
                          else "not plugged in (set FLIPPER in main.py)")
                elif name == "A":
                    slow = not slow
                    print("POWER:", f"{SLOW_POWER:.0%} (slow)" if slow else "100%")
                elif name == "UP" and backward_mode:
                    test_step, test_loops = (0, 0) if test_step is None else (None, 0)
                    print("TEST started" if test_step is not None else "TEST stopped")
                elif name in ("LEFT", "RIGHT", "DOWN"):
                    key = "reverse" if backward_mode else "forward"
                    if name == "LEFT":
                        balance[key] = max(-BALANCE_LIMIT, balance[key] - BALANCE_STEP)
                    elif name == "RIGHT":
                        balance[key] = min(BALANCE_LIMIT, balance[key] + BALANCE_STEP)
                    else:
                        balance[key] = 0.0
                    balance[key] = round(balance[key], 2)
                    save_balance(balance)
                    print(f"BALANCE {'backward' if backward_mode else 'forward'}: {describe(balance[key])}")
            was_down[name] = down

        # ----- thrusters -----
        power = SLOW_POWER if slow else 1.0
        # THROTTLE_STICK (left, up/down) = forward/backward, STEER_STICK (right, left/right) = turn.
        # Turn right: left side forward, right side backward (and the opposite for left).
        # Differential thrust: turn RIGHT = left side more forward, right side more backward.
        forward = THROTTLE_SIGN * read_stick(THROTTLE_STICK)
        turn = STEER_SIGN * read_stick(STEER_STICK)
        left = max(-100.0, min(100.0, forward + turn)) * power
        right = max(-100.0, min(100.0, forward - turn)) * power
        left, right = balanced(left, right, balance)

        if test_step is not None:
            if test_loops == 0:
                if test_step >= len(TEST_STEPS):
                    test_step = None
                    print("TEST finished")
                else:
                    name, ports, duty, seconds = TEST_STEPS[test_step]
                    test_loops = seconds * 50
                    if duty:
                        print(f"TEST {name} ({ports[0]}, REVERSED={REVERSED[ports[0]]}): "
                              "FORWARD command - water must go BACKWARD. "
                              f"If it goes forward, flip REVERSED['{ports[0]}']")
            if test_step is not None:
                _, ports, duty, _ = TEST_STEPS[test_step]
                for p in ALL_THRUSTERS:
                    set_thruster(p, duty if p in ports else 0.0)
                left_now = right_now = 0.0
                test_loops -= 1
                if test_loops == 0:
                    test_step += 1
        else:
            left_now += max(-RAMP_STEP, min(RAMP_STEP, left - left_now))
            right_now += max(-RAMP_STEP, min(RAMP_STEP, right - right_now))
            drive_sides(left_now, right_now)

        # ----- gripper: hold Y to open, B to close (skipped when GRIPPER is None) -----
        step = SERVO_SPEED / 50
        sign = 1 if GRIPPER_OPEN_DEG >= GRIPPER_CLOSED_DEG else -1
        if c.button("Y").is_down:
            gripper += sign * step
        elif c.button("B").is_down:
            gripper -= sign * step
        low, high = sorted((GRIPPER_CLOSED_DEG, GRIPPER_OPEN_DEG))
        gripper = max(low, min(high, gripper))
        if GRIPPER:
            robot.servo(GRIPPER).angle(gripper)
        moving = c.button("Y").is_down or c.button("B").is_down
        if GRIPPER and gripper_moving and not moving:
            print(f"GRIPPER stopped at {gripper:.0f} deg")  # use this to set the OPEN / CLOSED angles
        gripper_moving = moving

        # ----- flipper -----
        set_flipper(FLIPPER_DOWN_DEG if flipper_down else FLIPPER_UP_DEG)

        # ----- status, once a second -----
        loop += 1
        if loop % 50 == 0:
            raw = [c.joystick(n).value for n in ("ONE", "TWO", "THREE", "FOUR")]
            print(f"sticks 1={raw[0]:4.0f} 2={raw[1]:4.0f} 3={raw[2]:4.0f} 4={raw[3]:4.0f} | "
                  f"left={left_now:4.0f} right={right_now:4.0f} | gripper={gripper:3.0f} "
                  f"flipper={'DOWN' if flipper_down else 'UP'} | {'slow' if slow else 'full'}")


robot.run(
    initialize=initialize,
    autonomous=autonomous,
    driver=driver,
)
