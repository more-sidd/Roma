# Roma — 6-DOF robot arm (Feetech STS3215)

## Hardware
- **Controller link:** Waveshare Bus Servo Adapter (A), USB serial chip CH343 → was **COM5** on this Windows laptop, but the port can change, so the scripts **auto-detect** it by USB vendor ID `0x1A86` (WCH). COM3/COM4 are Bluetooth serial links, not the adapter.
  - Jumper must be in **position A** (USB ↔ servo bus). Position B routes USB to the onboard ESP32, and the servos go silent.
  - USB powers only the serial chip. **Servo power comes only from the board's DC input.**
- **Servos:** Feetech **STS3215, 7.4 V version, 1:147 gear ratio**, daisy-chained on one half-duplex TTL bus.
  - Supply: **5–7.4 V**, never above ~8.4 V. 5 V works with reduced torque.
  - Full arm: regulated 6–7.4 V supply, **≥5 A** (10 A preferred). One unloaded servo can run from a ≥2 A USB charger through a USB-to-DC barrel cable.
  - Encoder is on the output shaft: **4096 steps/rev**, about 11.38 steps per degree. Center = 2048. The gear ratio doesn't affect the code.
- **Bus:** 1,000,000 baud (default).
- **IDs found:** 1–7 (seven servos, though the arm is described as 6-DOF; ID 7 may be the gripper). Not confirmed with the user yet.

## Incident log
- **2026-10-14:** Proper supply connected (7.0 V). **ID 2 confirmed alive** on its own (pos 2229, 28 °C) and moved +1° and back. The first test FAILed only because a reply packet dropped. The script now retries and judges by measured position. After the fixes, ID 2 **passes**. The other IDs haven't been checked yet.
- **ID 2 behaviour:** it settles **3–5 steps (~0.3–0.45°) short** of every target, whatever the move size (tested 1° and 5°). This is the normal deadband: P=32, I=0, D=32, deadzone 1/1, and 1:147 gear friction. Repeated out-and-back tests creep the start position by a few steps. Homing offset (reg 31) = 2533, so it was calibrated before, probably by LeRobot. Angle limits are 0–4095 (none set).
- **2026-10-11:** The servo bus was powered by a **19.5 V laptop charger** (the servos read 19.4–19.6 V). The first scan worked. Afterward no servo responded at any baud, even with the jumper in A. The charger has been disconnected. **The servos and/or the adapter may be damaged.** Check each servo one at a time on a correct 5–7.4 V supply before trusting the chain.

## Snapshot from the first scan (2026-10-11, torque off, no load)
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
- `scan_servos.py [PORT|auto] [BAUD]`: port is auto-detected via `find_port()` when omitted. **read-only**. Pings IDs 0–253 and prints position, load, voltage and temperature for each servo found. Always run it first.
- `test_servos.py [--port COMx] [--ids ...] [--deg 1.0] [--speed 200] [--max-volts 8.4]`: nudges each servo +DEG and back, one at a time, and waits for Enter before each one.
  - Aborts before any motion if any servo reads above `--max-volts` (default 8.4 V).
  - Sets the goal to the current position before enabling torque, so nothing jumps.
  - Restores each servo's original torque state in a `finally` block.
  - `settle()` waits until the position has been stable for 5 reads (servo stopped), then compares. **Don't** go back to 'pass as soon as within tolerance': it passes mid-move.
  - Passes if the servo ends up within 6 steps (~0.5°) of the target, judged by **measured position**, not write acks. Prints how far the servo actually moved compared to the commanded move.
  - Retries each read/write up to 3× (`RETRIES`), since replies can drop when the motor first energizes.
- `test_servos.py` imports the protocol helpers from `scan_servos.py`. Keep `scan_servos.py` free of side effects at import time (argv parsing lives in `main()`).

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
1. Power the bus from a proper 5–7.4 V supply and re-scan. If anything is missing, test the servos one at a time.
2. Run `test_servos.py` to confirm each joint moves.
3. Confirm the role of ID 7, then map IDs to joints.
4. Calibrate joint limits and the home pose. After that: jogging, angle control, IK, record and replay.
