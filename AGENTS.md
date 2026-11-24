# Roma — SO-101 follower arm (TheRobotStudio SO-ARM100 repo), 6 × Feetech STS3215

Standalone follower arm, no leader arm. The wrist camera mount isn't fitted yet. The plan is to calibrate and learn the joints with these small scripts now, then move to LeRobot (`so101_follower`) for recording data and training policies.

## Hardware
- **Controller link:** Waveshare Bus Servo Adapter (A), USB serial chip CH343. The port has been **COM5** and later **COM16** on this Windows laptop, so the scripts **auto-detect** it by USB vendor ID `0x1A86` (WCH). COM3/COM4 are Bluetooth serial links, not the adapter.
  - Jumper must be in **position A** (USB ↔ servo bus). Position B routes USB to the onboard ESP32, and the servos go silent.
  - USB powers only the serial chip. **Servo power comes only from the board's DC input.**
- **Servos:** Feetech **STS3215 C046, 7.4 V version, 1:147 gear ratio, on ALL six joints** (the user confirmed this 2026-11-23). They're daisy-chained on one half-duplex TTL bus.
  - The standard SO-101 follower uses 1:345 (C001) motors. The 1:147 version is faster but has much less torque.
  - Expect shoulder_lift (ID 5), and sometimes elbow_flex (ID 4), to stall or sag under gravity. `test_servos.py` reports this as STALLED with the load %.
  - The ratio does **not** change steps per degree: the encoder is on the output shaft.
  - Supply: **5–7.4 V**, never above ~8.4 V. 5 V works with reduced torque.
  - Full arm: regulated 6–7.4 V supply, **≥5 A** (10 A preferred). One unloaded servo can run from a ≥2 A USB charger through a USB-to-DC barrel cable.
  - Encoder is on the output shaft: **4096 steps/rev**, about 11.38 steps per degree. Center = 2048. The gear ratio doesn't affect the code.
- **Bus:** 1,000,000 baud (default).

## Joints: exactly 6 motors, IDs 1–6, numbered gripper → base
The SO-101 has **six motors and no more**: five arm joints plus the gripper.

**ID order on this arm is gripper → base: gripper = 1 … shoulder_pan (base) = 6.** The user confirmed on 2026-11-23 that every motor's internal ID now matches this. It's the single source of truth: `JOINTS` in `sts_bus.py` and `scan_servos.py` encodes it. Don't renumber the motors or change `JOINTS` without the user asking.

| ID | Joint | Motion |
|---|---|---|
| 1 | gripper | jaw open/close |
| 2 | wrist_roll | wrist twist |
| 3 | wrist_flex | wrist pitch |
| 4 | elbow_flex | forearm bend |
| 5 | shoulder_lift | upper arm up/down (heaviest load) |
| 6 | shoulder_pan | base rotation, left/right |

**Any ID outside 1–6 is a joint with the wrong ID, never an extra motor.** Renumber it, don't treat it as a 7th servo.

**LeRobot note:** the joint names are LeRobot's, but stock `so101_follower` numbers the other way round (shoulder_pan = 1 … gripper = 6). Before moving to LeRobot, either renumber the motors with `so101_setup_ids.py` (only if the user asks), or give LeRobot this arm's motor map. Otherwise every command goes to the wrong joint.

### Joint limits (design values, SO-101 URDF `so101_new_calib.urdf`)
0° is the calibrated middle of each joint's range. At 11.38 steps/deg, the calibrated motor window should be roughly 2047 ± (limit × 11.38).

| ID | Joint | Min | Max | Travel |
|---|---|---|---|---|
| 1 | gripper | −10.0° | +100.0° | 110° |
| 2 | wrist_roll | −157.2° | +162.8° | 320° |
| 3 | wrist_flex | −95.0° | +95.0° | 190° |
| 4 | elbow_flex | −96.8° | +96.8° | 194° |
| 5 | shoulder_lift | −100.0° | +100.0° | 200° |
| 6 | shoulder_pan | −110.0° | +110.0° | 220° |

The limit registers in the motors are still at the factory 0–4095 (no limits). Measure the real ranges with `joint_limits.py --measure`. `so101_calibrate.py` stores them in the motors.

### Bus state, 2026-11-23 (COM16, 7.0 V, torque off)
This is a history log. Joint names in it **before the final "Order confirmed" entry** use the old numbering (shoulder_pan = 1 … gripper = 6), which turned out not to match the hardware. For the current mapping, see the table above.
- **Answering:** IDs 1, 2, 5, 6 and **7**.
- **Missing:** IDs 3 and 4. They weren't found at any ID (0–253) or any baud rate.
- Fifty repeated reads per ID were clean, so no two motors share an ID.
- **Conclusion:** ID 7 is either elbow_flex or wrist_flex with the wrong ID. The other of the two is **off the bus**: unplugged, a bad cable, or possibly killed by the 19.5 V incident below.
- Every motor still has old homing offsets from an earlier calibration. The limits are 0–4095.
- **Later the same day:** after reconnecting, **ID 4 (wrist_flex) answers too**. That makes 6 motors on the bus: IDs 1, 2, 4, 5, 6, 7.
  - So ID 7 is almost certainly **elbow_flex**. Confirm with `so101_scan.py COM16 --watch` by moving the elbow.
  - The only ID problem left is renumbering 7 → 3.
- **Fixed, 2026-11-23:** ran `so101_setup_ids.py COM16 --change 7 3`, which wrote the new ID to the motor's EEPROM with the lock off. The re-scan shows **IDs 1–6 all present, nothing else on the bus**.
- **Power-cycle check, 2026-11-23:** after the servo supply was unplugged and replugged, the bus still shows IDs 1–6 with elbow_flex at 3. **The ID change is permanent.**
- **User nudge tests, 2026-11-23** (`test_servos.py` before the fixes below):
  - IDs 3, 4 and 5: OK.
  - ID 1: moved only partway (+27 of +57 steps).
  - ID 2: crashed with `OverflowError`. Its position read negative because of the old homing offset, and the script couldn't send a negative goal.
  - ID 6: didn't move at all (+0 of +11).
  - ID 5 at +10° and +30°: moved only 19 steps, because the script clamped the goal at 4095.
- **ID swap 1 ↔ 3, 2026-11-23**, at the user's request. It went 1 → 201, 3 → 1, 201 → 3.
  - Verified: the old ID 1 motor (pos 3069) now answers as ID 3, and the old ID 3 motor (pos 1156) now answers as ID 1.
  - The bus shows IDs 1–6. IDs 2, 4, 5 and 6 weren't changed. Whether this fully fixes the order hasn't been confirmed yet.
- **ID swap 2 ↔ 4, 2026-11-23**, at the user's request. It went 2 → 202, 4 → 2, 202 → 4.
  - Verified by homing offset, which stays with the physical motor: the old ID 4 motor (offset −1737) now answers as ID 2, and the old ID 2 motor (offset −485) now answers as ID 4.
  - The bus shows IDs 1–6. IDs 1, 3, 5 and 6 weren't changed by this swap.
- **ID swap 3 ↔ 4, 2026-11-23**, at the user's request. It went 3 → 203, 4 → 3, 203 → 4.
  - Verified by homing offset: the motor with offset −485 is now ID 3, and the motor with offset −1795 is now ID 4.
  - The bus shows IDs 1–6.
- **ID swap 5 ↔ 6, 2026-11-23**, at the user's request. It went 5 → 205, 6 → 5, 205 → 6.
  - Verified by homing offset: the motor with offset −1625 is now ID 5, and the motor with offset −1772 is now ID 6.
  - The bus shows IDs 1–6.
  - **ID map after all four swaps (1↔3, 2↔4, 3↔4, 5↔6):** ID 1 = offset 208, ID 2 = −1737, ID 3 = −485, ID 4 = −1795, ID 5 = −1625, ID 6 = −1772.
- **Order confirmed, 2026-11-23:** the user confirmed every motor's internal ID is now correct, **numbered gripper → base: 1 gripper, 2 wrist_roll, 3 wrist_flex, 4 elbow_flex, 5 shoulder_lift, 6 shoulder_pan**.
  - `JOINTS` in all scripts was changed to match.
  - By old homing offset (until recalibration): gripper = 208, wrist_roll = −1737, wrist_flex = −485, elbow_flex = −1795, shoulder_lift = −1625, shoulder_pan = −1772.
  - No `--identify` run is needed.
- Positions can read **below 0 or above 4095** while the old offsets are in place (one joint read −712). Every script now decodes them as signed values (bit 15).
- **Still to do:**
  1. Run `joint_limits.py --measure`.
  2. Calibrate.

## Incident log
- **2026-10-14:** Proper supply connected (7.0 V). **ID 2 confirmed alive** on its own (pos 2229, 28 °C) and moved +1° and back. The first test FAILed only because a reply packet dropped. The script now retries and judges by measured position. After the fixes, ID 2 **passes**. The other IDs haven't been checked yet.
- **ID 2 behaviour:** it settles **3–5 steps (~0.3–0.45°) short** of every target, whatever the move size (tested 1° and 5°). This is the normal deadband: P=32, I=0, D=32, deadzone 1/1, and 1:147 gear friction. Repeated out-and-back tests creep the start position by a few steps. Homing offset (reg 31) = 2533, so it was calibrated before, probably by LeRobot. Angle limits are 0–4095 (none set).
- **2026-10-11:** The servo bus was powered by a **19.5 V laptop charger** (the servos read 19.4–19.6 V). The first scan worked. Afterward no servo responded at any baud, even with the jumper in A. The charger has been disconnected. **The servos and/or the adapter may be damaged.** Check each servo one at a time on a correct 5–7.4 V supply before trusting the chain.

## Snapshot from the first scan (2026-10-11, torque off, no load)
This is historical. Seven IDs answered here, but the SO-101 has only 6 motors. So either a seventh, non-arm servo was on the bus then, or the readings were unreliable at 19.5 V. Don't read this table as "the arm has 7 servos".
| ID | Pos | Deg | Temp |
|---|---|---|---|
| 1 | 2015 | 177.1 | 27 °C |
| 2 | 2047 | 179.9 | 27 °C |
| 3 | 2045 | 179.7 | 27 °C |
| 4 | 2050 | 180.2 | 26 °C |
| 5 | 2012 | 176.8 | 26 °C |
| 6 | 1057 | 92.9 | 28 °C |
| 7 | 895 | 78.7 | 31 °C |

## Scripts (Python 3.12, `pyserial` only; no Feetech SDK)
- `scan_servos.py [PORT|auto] [BAUD]`: port is auto-detected via `find_port()` when omitted. **read-only**.
  - Pings IDs 0–253, so a misnumbered motor still shows up.
  - Prints the joint name, position, load, voltage and temperature for each servo found.
  - Ends by listing missing joints (1–6) and flagging any ID outside 1–6 as misnumbered.
  - Always run it first.
  - The joint map `JOINTS` (gripper = 1 … shoulder_pan = 6) lives here for `test_servos.py` and `joint_limits.py`. An identical copy is in `sts_bus.py` for the `so101_*` tools. **Keep the two in sync.**
- `test_servos.py [--port COMx] [--ids ...] [--deg 1.0] [--speed 200] [--max-volts 8.4]`: nudges each servo +DEG and back, one at a time, and waits for Enter before each one.
  - The default IDs are the 6 joints, 1–6. To nudge a misnumbered motor, pass its ID explicitly, e.g. `--ids 7`. A negative `--deg` nudges the other way.
  - Lists any requested ID that doesn't respond.
  - Goals are sent sign-magnitude (bit 15), so negative positions work.
  - **Skips a joint, without moving it**, if the start or the target falls outside the motor's Min/Max_Position_Limit window. Otherwise the motor would clamp the goal and snap the joint to the window edge.
  - Reports **STALLED** when the joint stops short while load is ≥25%: gravity, a hard stop, or the low torque of the 1:147 motors. It reports FAIL when it's short with little load.
  - Aborts before any motion if any servo reads above `--max-volts` (default 8.4 V).
  - Sets the goal to the current position before enabling torque, so nothing jumps.
  - Restores each servo's original torque state in a `finally` block.
  - `settle()` waits until the position has been stable for 5 reads (servo stopped), then compares. **Don't** go back to 'pass as soon as within tolerance': it passes mid-move.
  - Passes if the servo ends up within 6 steps (~0.5°) of the target, judged by **measured position**, not write acks. Prints how far the servo actually moved compared to the commanded move.
  - Retries each read/write up to 3× (`RETRIES`), since replies can drop when the motor first energizes.
- `joint_limits.py [--port COMx] [--measure]`: **read-only**.
  - Prints the URDF design limits and each motor's current limit window, by ID (gripper first).
  - With `--measure` (torque off), it records the real min/max as you move each joint by hand and compares the spans to the URDF.
- `arm_ui.py [--port COMx] [--http 8101] [--sim]` + `arm_ui.html`: **local web control panel**. Open http://127.0.0.1:8101. It uses only stdlib + pyserial, and imports `sts_bus`.
  - **Layout:** one card per motor, in ID order (gripper = 1 … shoulder_pan = 6).
  - **Each card:** a torque switch; a position slider (goal = thumb, present = tick mark); jog buttons ±1° and ±5°; live load %, temperature, supply voltage and speed. Degrees are shown as (pos − 2047) / 11.38. That's only a real joint angle after calibration.
  - **Top bar:** speed (2–120 °/s, default 20), Torque on all, Hold all (Space), and Torque off all (Esc). The arm goes limp with Torque off all.
  - **Safety, server-side:**
    - Torque-on is refused if voltage is above 8.4 V, or if the joint is more than ~2° (`EDGE_TOL`) outside its limit window. Within that margin, the goal is clamped to the window edge, since a limp joint often rests on its hard stop just past the recorded range.
    - Torque-on sets goal = present first, so nothing jumps.
    - Goals are refused while torque is off, and clamped into the window.
    - Acceleration ramp is 40 (×100 steps/s²).
  - **Calibration tab:** the same procedure as `so101_calibrate.py` and `lerobot-calibrate`, run as a server-side state machine (`idle → posing → recording → done`). Open it directly at `#calibration`.
    - **Start** (needs all 6 online, plus an inline confirm): torque off and lock off, Operating_Mode 0, Homing_Offset 0, limits 0–4095. It first saves each motor's old offset and limits.
    - **Set middle:** Homing_Offset = raw − 2047, clamped to ±2047.
    - **Record:** min/max come from the poll loop while you move the joints by hand. **Save** needs at least 100 steps of travel per joint, with min/max inside 0–4095.
    - **Save** writes the min/max to the limit registers and the JSON to the LeRobot cache dir and `calibration/<id>.json`.
    - **Cancel** restores the old offsets and limits exactly.
    - Torque-on, goals and Torque on all are **refused while a calibration is running**.
    - The robot id must match `[A-Za-z0-9_-]{1,40}`, since it becomes the file name.
    - **Two modes:** "All joints together" or **"One joint at a time"**. `cal_start` takes `ids`; it calibrates all joints when `ids` is omitted.
      - Per-joint mode resets and turns torque off **only on that joint**. The other joints keep torque and settings, and still accept control commands, so torque can hold the arm up.
      - Locks are per joint: `_calibrating(i)` blocks torque-on and goals only for joints being calibrated. Torque on all skips them.
      - The picker lists each joint's status (calibrated means the limit window isn't 0–4095) and highlights the next one to do.
      - **Save merges:** new entries for the calibrated joints, plus the other joints' entries from the existing `<id>.json`. If a joint has no entry, it's taken from its motor registers when they hold a real window. Joints never calibrated are left out, so the file can be partial until all 6 are done, and the Done screen says "N of 6".
      - Cancel restores only the joints in that run.
    - **Middle-pose helper (both modes):** during step 2, the poll loop records the range seen so far (`seen_min`/`seen_max`). The page shows it as a shaded bar with a ◆ at its midpoint, plus a "To middle" column that turns green within ±5°. There's also a hint for each joint.
    - **Old-server guard:** if the server's state has no `cal.ids` (an `arm_ui.py` from before per-joint mode, which ignores `ids` and would reset ALL joints), the page disables per-joint mode and says to restart.
  - **Motor settings check** (Calibration tab, bottom): `GET /api/settings` is a **read-only** dump of registers 0–69 from every online motor, taken under the bus lock.
    - The page shows the key registers side by side, highlights values that differ from the majority, and flags multi-turn causes: Operating_Mode ≠ 0, Angular_Resolution (reg 30) ≠ 1, both limits 0, or a raw reading (present + offset) outside 0–4095.
    - **2026-11-23:** after the user calibrated gripper, elbow_flex, shoulder_lift and shoulder_pan into `roma.json`, **wrist_roll (ID 2) and wrist_flex (ID 3) wouldn't calibrate.** With Homing_Offset 0 and window 0–4095 they read **−10349** and **4631**, i.e. counting beyond one turn.
      - **The full register dump (0–69) showed every configuration register identical on all 6 motors:** Mode 0, Angular_Resolution 1, Phase 92, same PID, limits and protections, firmware 3.10.
      - **Cause:** the STS3215 keeps a turn count in RAM when a joint passes the sensor's 4095→0 point. Present = continuous count − offset, with no wrap to 0–4095. Homing_Offset is limited to ±2047, so a reading more than about half a turn outside 0–4095 can't be centred, and Set middle produced garbage. The count clears on power-up.
      - **Guard (2026-11-23):** `cal_start` refuses (and restores the registers) if a joint reads outside 0–4095 with offset 0. `cal_set_middle` refuses the same way. The posing table warns as soon as a joint passes the zero point.
      - **Procedure for wrist joints:** power-cycle the servos, start, move the joint **straight to its middle without sweeping to the ends first**, then Set middle. Centring puts the zero point opposite the middle, outside the range.
      - **Confirmed by power cycle (2026-11-23):** wrist_roll went from −10349 to **1940**, and wrist_flex from 4379 to **1327**. The counter clears on power-up. All 6 raw readings were inside 0–4095.
      - **Side effect:** while limp, elbow_flex (3071 vs max 3067) and shoulder_lift (1055 vs min 1061) rested on their hard stops just past the recorded range. Torque-on now accepts a joint up to `EDGE_TOL` = 23 steps (~2°) outside its window and eases it to the edge. Beyond that it still refuses.
      - Wrist joints still need calibrating, via per-joint mode, going straight to the middle.
  - **Live indicator:** the header shows `live N Hz` from the poll loop's heartbeat (`state.poll`). If the readings stop for more than 1 s, a red banner appears. The poll loop catches every exception, reports it, and keeps going, so it can't silently freeze.
  - **`--sim`:** a simulated bus for trying the UI without hardware. `.claude/launch.json` has an `arm-ui-SIMULATOR-not-real-arm` entry on port 8102.
    - **2026-11-23 incident:** the user did step 3 in the browser pane on the 8102 simulator while their real server ran on 8101, so moving the arm by hand changed nothing on screen.
    - Since then, sim mode shows a purple "SIMULATOR — not connected to your arm" banner on every tab, and the header and title say SIMULATOR.
    - **Agents:** stop the sim preview when you're done testing, and never leave it as the pane's active tab.
    - It starts with the real arm's leftover offsets, so one joint reads above 4095, as it does on the real arm.
    - It models the offset and the motor's goal clamp, and has a "Simulate moving the joints" sweep button in the range step.
    - It saves only to `calibration/sim/` (gitignored), **never** to the LeRobot cache.
  - **Tested 2026-11-23:** end-to-end in `--sim`, covering torque, jog, slider, clamping, Hold, Esc and refusals. The full calibration flow was also tested, with cancel-restore, the locks and the Control tab picking up the new limits. Per-joint mode was tested in the sim: only the chosen joint was reset, the others kept torque and accepted goals, the ◆ midpoint was correct, two saves merged into one file, and cancel restored only that joint. The old-server guard wasn't exercised against a real old server. Also started against the real COM16 read-only, but the servos were unpowered, so they showed as offline. **Not yet used to move the real arm.**
  - Ctrl+C stops the server, and the motors keep their torque state. Use Torque off all first if you want the arm limp.
- `remote.py` (used by `arm_ui.py`, **Remote tab**, added 2026-11-23): DualShock 4 control through `pygame` (optional dependency; the UI says so if it's missing).
  - Per-joint mode: **active** (stick drives it), **hold** (torque on, not driven), **limp** (torque off; joints 4/5 sag). Defaults: 1–3 active, 4–6 hold, so the loaded lower joints aren't driven.
  - **Controls (changed 2026-11-23):** **no deadman** (the user removed it). Joints move only while a stick/button is pushed and stop when released. **X = close gripper, O = open, L1 / L2 = wrist roll**, D-pad ↑/↓ = speed scale, PS = hold all, Options = limp all + remote off. Sticks as below.
  - **Arming:** after Start remote mode or a pad reconnect, nothing moves until every stick, L2 and the X/O/L1 buttons have been seen at rest (so enabling can't jump). Triggers that report 0 instead of −1 until touched are ignored until seen at rest. Losing the pad, or 0.5 s without input, calls `hold_all`.
  - The Remote tab table shows each joint's **load %** (yellow ≥25, red ≥60) and temperature live.
  - Heat guard: an active joint ≥55 °C becomes hold; any joint ≥65 °C goes limp.
  - It goes through `Arm.goal()`/`torque()`, so voltage, window and calibration-lock rules still apply. The goal never leads the measured position by more than 12°.
  - **Two styles.** Joint style: sticks drive joints directly. **Cartesian style** (`kinematics.py`, built 2026-11-23): left stick = tool forward/back and left/right, right stick Y = up/down, right stick X = tool pitch; L1/L2 roll and X/O gripper stay direct.
    - `kinematics.py` is FK + damped-least-squares IK in pure Python, with the geometry copied from the official `so101_new_calib.urdf`. It solves [x, y, z, pitch] over IDs 6, 5, 4, 3 (pan, lift, elbow, wrist_flex). At the zero pose the tool is at about x = 0.39 m, z = 0.23 m, pitch 0.
    - Only **Active** joints among IDs 3–6 move. **Hold** joints stay fixed, so the IK just has fewer joints; **limp** joints on 3–6 are refused in Cartesian.
    - **Refused until calibrated:** all six motors online, a saved calibration file, and a calibrated limit window on IDs 3–6. The Remote tab says why.
    - The target may lead the tool by at most 3 cm and the goal may lead the measured joint by at most 12°. An unreachable target is pulled back, so it can't wind up. Each joint's goal speed is paced to its commanded rate so the path stays straight (`Arm.goal(..., dps=)`).
    - `DIR` (motor direction vs URDF angle) was checked by hand on the real arm 2026-11-23 with `dir_check.py` (read-only, torque off, needs `arm_ui.py` running): **ID 5 shoulder_lift is inverted (−1)**; IDs 6, 4, 3 are +1. wrist_roll (2) isn't used by the IK and wasn't checked. **Cartesian has not yet moved the real arm.**
  - **DS4 axis/button indices (`AX`, `BTN`) verified on the real pad 2026-11-23** with `pad_check.py` (read-only; `python pad_check.py [sticks]`): sticks 0–3 (X right = +1, Y down = +1), L2/R2 = axes 4/5, L1 = 9, PS = 5, Options = 6, D-pad = buttons 11–14.
  - **Joint direction signs (`SIGN`, incl. gripper: O = +, and roll: L1 = +) are still unverified** (they need the arm). **Not yet used on the real arm.**
  - Tested 2026-11-23 in `--sim` with a fake pad (deadman, pad loss, modes, window clamp, heat guard, Options, calibration lock; Cartesian: straight forward/up moves, hold joints fixed, leash, gating). IK round trip: 200 random poses, 0 failures.
- `test_servos.py` imports the protocol helpers from `scan_servos.py`. Keep `scan_servos.py` free of side effects at import time (argv parsing lives in `main()`).
- **Setup and calibration tools** (same folder). Each takes the port as an argument, e.g. `COM16`.
  - `sts_bus.py`: the shared protocol module, with its own copy of `JOINTS`.
    - `read_raw` / `write_raw` do block reads and writes. `encode` / `decode` handle sign-magnitude.
    - It also has the Acceleration, Goal_Time, Goal_Speed and Present_Speed registers.
  - `so101_scan.py PORT [--watch]`: read-only status, or a live joint readout with `--watch`.
  - `so101_setup_ids.py PORT --change OLD NEW | --one JOINT | --all | --identify`: writes the ID and sets the baud to 1M.
    - `--all` goes gripper (ID 1) first and shoulder_pan (ID 6) last.
    - `--identify` works on the assembled arm, with torque off. You wiggle each joint gripper → base, and it finds the motor that moved. It's not needed now that the order is confirmed.
    - It prints the current-ID → new-ID table. It only writes after you type `YES`.
    - It swaps through temporary IDs 201–206, so two motors never share an ID.
    - It was tested in simulation with a fully reversed order, not yet on the arm.
  - `so101_calibrate.py PORT --id my_follower`: works like `lerobot-calibrate`.
    - Homing_Offset = mid-pose position − 2047.
    - It records the range min/max and writes them to the limit registers.
    - It saves LeRobot-format JSON to `~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<id>.json` and to `calibration/`.
    - Each joint's `"id"` in that JSON is this arm's ID (gripper = 1 … shoulder_pan = 6). That's the reverse of stock LeRobot; see the LeRobot note under Joints.

## STS protocol cheat-sheet
Packet: `FF FF ID LEN INSTR PARAMS… CHK`, where `LEN = len(params)+2` and `CHK = ~(ID+LEN+INSTR+sum(params)) & 0xFF`.
Instructions: PING `0x01`, READ `0x02` (params: addr, n), WRITE `0x03` (params: addr, data…).
Multi-byte values are little-endian. Position uses bit 15 as the sign bit and load uses bit 10.

| Addr | Size | Register |
|---|---|---|
| 40 | 1 | Torque enable |
| 41 | 1 | Acceleration |
| 42 | 2 | Goal position |
| 44 | 2 | Goal time (0 = use speed) |
| 46 | 2 | Goal speed (steps/s) |
| 55 | 1 | EEPROM lock |
| 56 | 2 | Present position |
| 60 | 2 | Present load |
| 62 | 1 | Present voltage (×0.1 V) |
| 63 | 1 | Present temperature (°C) |
| 66 | 1 | Moving flag |
| 69 | 2 | Present current |

EEPROM (read so far, not changed): 9/11 min/max angle (2 B), 21 P, 22 D, 23 I, 24 min start force (2 B), 26/27 CW/CCW deadzone, 31 homing offset (2 B), 33 mode (0 = position), 48 torque limit (2 B).

## Rules for agents
- Never command motion without checking bus voltage first (≤ 8.4 V).
- Scanning and reads are safe. Anything that writes (torque, goal position, EEPROM, IDs) moves or changes hardware, so get the user's go-ahead.
- Move slowly and in small steps until joint limits are calibrated. Joint limits and the home pose are **not yet defined**.

## Next steps
1. ~~Renumber the elbow, 7 → 3.~~ Done 2026-11-23. All 6 IDs answer.
2. ~~Power-cycle and re-scan.~~ Done, the IDs persist.
3. ~~Fix the physical ID order.~~ Done 2026-11-23 by manual swaps. The user confirmed gripper = 1 … shoulder_pan = 6.
4. Calibrate, either on the `arm_ui.py` Calibration tab or with `so101_calibrate.py`. Both produce the same result. `joint_limits.py --measure` is optional if you only want to look.
5. After calibration, every joint sits near 2047 with real limits. Then re-run `test_servos.py`: the skips go away, and STALLED shows which joints the 1:147 motors struggle with. After that: jogging, angle control, IK.
6. Then LeRobot for recording and training, once the camera is mounted. First sort out the reversed ID order (see the LeRobot note under Joints).
