# Trident-X 2026 — MARLIN boat code

Driver and autonomous code for our MARLIN Robotics *Link Lockdown* boat (Trident-X 2026).

Open this folder in VS Code with the MARLIN Controller extension installed (the folder that
contains `project.marlin`), connect the Controller and Robot Brain, then press **Driver** or **Auton**.

## Files

| File | What it is |
|---|---|
| `src/main.py` | Robot code: 4 thrusters, gripper, flipper, balance, slow mode, thruster test |
| `src/auton.py` | Autonomous Period: find the Beacon by radio, grab it, bring it to our Starting Area |
| `tools/sim_auton.py` | Runs `auton.py` against a simulated pool (`python tools/sim_auton.py`) |
| `project.marlin` | Marks this as a MARLIN project; also stores the joystick calibration |
| `balance.json` | Fan balance tuned with the D-pad (loaded automatically) |
| `logs/` | Written after every Auton run: every Beacon message and a 50 Hz trace (not committed) |

## Autonomous (first 20 s)

There are two modes. Pick one before **every** match at the top of `src/auton.py`.

### Mode 1: BEACON SEEN (use this whenever the Beacon is in the pool before we set the boat down)

1. Measure where the Beacon sits on the Centerline: metres sideways from the middle of the pool, seen
   from behind our wall, `+` = right, `-` = left (always about 0.22 to 0.75 m out). Tape marks every
   5 cm along our wall make this quick.
2. Put it in `BEACON_SEEN_Y`, e.g. `BEACON_SEEN_Y = 0.47`.
3. Start the program. It prints exactly where to put the boat, for example:
   `AUTON MODE: BEACON SEEN at 0.47 m right. Put the boat SIDEWAYS against our wall, centre 0.22 m right
   of the middle of our wall, bow pointing RIGHT.`
4. After the match set it back to `None` (or the next match's value).

The boat curves onto the Beacon's line, drives straight in with the gripper open, closes it and
reverses straight back to our wall, with no turn while it holds the Beacon. No radio guessing.
Simulated: about 50 % scored (85 % with a jaw that grabs from 0.20 m), against 9 % for the radio search.
Measuring the Beacon to +/-5 cm is enough (51 %); +/-10 cm drops to 42 %.
Ask the Head Referee whether robots may be placed after the Beacon is in the pool.

**Radio "Beacon in the jaws" check (optional, off until measured):** with `CLOSE_RSSI` set, the gripper
also closes as soon as 2 radio messages in a row are at least that loud, whichever comes first with the
distance driven. To measure it: hold the Beacon in the open jaws, start the program and read `rssi=` in
the `SENSOR RADIO` line, then set `CLOSE_RSSI` a few dB below the quietest value. Simulated gain is
small (about +3 % with the speed measured, +7 % without), because most misses are sideways and signal
strength gives no direction.

### Mode 2: RADIO SEARCH (`BEACON_SEEN_Y = None`, when we can't see the Beacon)

Set the start pose at the top of `src/auton.py`:

- `START_HEADING`: which way the bow points. `+90` = along our wall to the right, `-90` = to the left,
  `0` = straight at the field. The boat is longer than the 0.4 m Starting Area, so it starts sideways.
- `START_Y`: where along our wall (`-0.45` = left end, `0` = middle, `+0.45` = right end).
  The best start is at one end of the wall with the bow pointing toward the other end.

What it does: it curves out to a line in front of the Anchorage and sweeps across the pool. Every radio
message is stored together with where the boat was when it arrived, and all of them are fitted to
"signal drops with distance" to estimate where on the Centerline the Beacon is. Then it lines up, goes
in, closes the arm and drives back bow first, pushing the Beacon against our wall until the period ends.

The radio only gives signal strength (`rssi`), and `read()` only holds the latest message, so the code
reads it every loop and uses `rx_count` to catch every message. The log reports how many were missed.

**Steering:** tank drive, the same as the sticks: both left thrusters (M3, M1) against both right
thrusters (M5, M7), using the balance saved with the D-pad.

**Calibrate in the pool first.** The checklist is at the bottom of `src/auton.py`. The most important
values are the speed and turn rate (`SPEED_MPS`, `TURN_RATE_DPS`), the arm angles (`ARM_OPEN_DEG`,
`ARM_CLOSED_DEG`), `CLOSE_RSSI`, and checking that the heading goes up when the boat turns right.

Simulated success rate (`tools/sim_auton.py`, guessed boat and radio): about 5-10 % with a grab zone
of 0.10 m around the arm, about 30 % with 0.20 m. A wide mouth on the arm matters more than tuning.

## Controller

| Control | Action |
|---|---|
| Stick ONE (left) up / down | Both LEFT thrusters (M3 front, M1 back) forward / back |
| Stick THREE (right) up / down | Both RIGHT thrusters (M5 front, M7 back) forward / back |
| Y (hold) | Open the gripper (M4); stays where you let go |
| B (hold) | Close the gripper |
| X (tap) | Flipper (M2) down / up |
| A (tap) | Slow mode on / off (60 % power) |
| D-pad LEFT / RIGHT | Setup: turn the left / right side down 2 % (forward balance, saved) |
| Right trigger + D-pad LEFT / RIGHT | Setup: same, for the backward balance |
| D-pad DOWN (+ right trigger) | Setup: reset the forward (backward) balance |
| Right trigger + tap UP | Setup: thruster test, each thruster alone at 30 % (tap again to stop) |

Sensors: IMU on S1, Radio Receiver on S6. At startup the output prints `SENSOR ... OK` or `NO DATA`
for each. Stick sensitivity (`FORWARD_GAIN`, `BACKWARD_GAIN`, `DEADZONE`), smoothing (`RAMP_STEP`),
ports and `REVERSED` thrusters are at the top of `src/main.py`.
