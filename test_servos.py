"""Nudge each SO-101 servo by a small angle, one at a time, then return it.

For each servo: read position -> enable torque -> move +DEG -> move back ->
restore the original torque state. Aborts if bus voltage is out of range.
Defaults to the arm's 6 joint IDs (1-6). A misnumbered motor can still be
tested by passing its ID explicitly, e.g. --ids 7. Use a negative --deg to
nudge the other way (e.g. when a joint sits at the top of its limit window).

Gear ratio: all six motors are STS3215 C046, 1:147. The magnetic encoder is
on the OUTPUT shaft, so 4096 steps = 360 deg of joint motion for any ratio;
the ratio only sets torque and speed. 1:147 is the low-torque, fast version
(the standard SO-101 follower uses 1:345), so a joint fighting gravity can
stall short of its target. That is reported as STALLED with the measured
load rather than as a plain FAIL.

Usage: python test_servos.py [--port COMx] [--ids 1 2 3] [--deg 1.0] [--max-volts 8.4]
"""
import argparse
import time

import serial

from scan_servos import (ADDR_LOAD, ADDR_POSITION, ADDR_VOLTAGE, JOINTS, PING, find_port, read,
                         signed, transact)

WRITE = 0x03
ADDR_MIN_LIMIT, ADDR_MAX_LIMIT = 9, 11
ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION = 40, 42
GEAR_RATIO = 147     # STS3215 C046 on every joint; informational, see docstring
STEPS_PER_REV = 4096  # output-shaft encoder, independent of GEAR_RATIO
STEPS_PER_DEG = STEPS_PER_REV / 360
TOLERANCE = 6  # steps (~0.5 deg); STS3215 settles 3-5 steps short (P-only control, gear friction)
STALL_LOAD = 250  # Present_Load is per-mille of max torque; >25% while short of target = stalled


RETRIES = 3  # replies occasionally drop when the motor first energizes


def write(ser, sid, addr, data):
    for _ in range(RETRIES):
        if transact(ser, sid, WRITE, bytes([addr]) + bytes(data)) is not None:
            return True
    return False


def read_retry(ser, sid, addr, n):
    for _ in range(RETRIES):
        if (raw := read(ser, sid, addr, n)) is not None:
            return raw
    return None


def read_u16(ser, sid, addr):
    raw = read_retry(ser, sid, addr, 2)
    return int.from_bytes(raw, "little") if raw else None


def position(ser, sid):
    raw = read_u16(ser, sid, ADDR_POSITION)
    return signed(raw, 15) if raw is not None else None


def load(ser, sid):
    raw = read_u16(ser, sid, ADDR_LOAD)
    return signed(raw, 10) if raw is not None else None


def encode_pos(steps):
    """Sign-magnitude, sign bit 15 (positions can be negative after a homing offset)."""
    return (-steps) | 0x8000 if steps < 0 else steps


def move(ser, sid, goal, speed):
    # goal position (2), goal time (2, 0 = use speed), goal speed (2)
    data = (encode_pos(goal).to_bytes(2, "little") + (0).to_bytes(2, "little")
            + speed.to_bytes(2, "little"))
    return write(ser, sid, ADDR_GOAL_POSITION, data)


def settle(ser, sid, goal, timeout=1.5, stable_reads=5):
    """Wait until the servo stops moving, then return its final position."""
    end = time.time() + timeout
    last, same = None, 0
    while time.time() < end:
        p = position(ser, sid)
        same = same + 1 if p is not None and p == last else 0
        last = p if p is not None else last
        if same >= stable_reads and abs(last - goal) <= TOLERANCE:
            break
        if same >= stable_reads * 4:  # stopped, but off target
            break
        time.sleep(0.02)
    return last


def check_window(sid, start, target, lo, hi):
    """The motor clamps every goal into [Min_Position_Limit, Max_Position_Limit].
    If the joint is already outside that window, enabling torque would snap it
    to the edge (possibly tens of degrees), so refuse instead of moving."""
    if lo == hi == 0:  # 0/0 means no limits (multi-turn style); nothing to check
        return None
    if not lo <= start <= hi:
        return (f"ID {sid}: position {start} is outside its limit window [{lo}, {hi}], so torque-on "
                f"would snap it to {lo if start < lo else hi}. Skipped. Calibrate first "
                "(so101_calibrate.py re-centres every joint near 2047).")
    if not lo <= target <= hi:
        return (f"ID {sid}: target {target} is beyond its limit window [{lo}, {hi}]. Skipped. "
                "Nudge the other way (flip the sign of --deg), or calibrate first.")
    return None


def test_servo(ser, sid, deg, speed):
    start = position(ser, sid)
    torque = read_retry(ser, sid, ADDR_TORQUE_ENABLE, 1)
    lo, hi = read_u16(ser, sid, ADDR_MIN_LIMIT), read_u16(ser, sid, ADDR_MAX_LIMIT)
    if None in (start, torque, lo, hi):
        print(f"ID {sid}: no response, skipped")
        return "NO REPLY"

    delta = round(deg * STEPS_PER_DEG)
    target = start + delta
    if (why := check_window(sid, start, target, lo, hi)):
        print(why)
        return "SKIPPED"

    loads = {}
    try:
        move(ser, sid, start, speed)  # hold current pose so enabling torque doesn't jump
        write(ser, sid, ADDR_TORQUE_ENABLE, [1])
        # Judge by measured position, not by whether the write was acknowledged.
        move(ser, sid, target, speed)
        reached = settle(ser, sid, target)
        loads["out "] = load(ser, sid)
        move(ser, sid, start, speed)
        back = settle(ser, sid, start)
        loads["back"] = load(ser, sid)
    finally:
        write(ser, sid, ADDR_TORQUE_ENABLE, torque)

    verdict = "OK"
    for label, frm, goal, got in (("out ", start, target, reached), ("back", reached, start, back)):
        ld = loads[label]
        if got is not None and abs(got - goal) <= TOLERANCE:
            res = "OK"
        elif got is not None and ld is not None and abs(ld) >= STALL_LOAD:
            res = "STALLED"  # pushing hard but can't get there: gravity, a hard stop, or 1:147 torque
        else:
            res = "FAIL"
        if res != "OK" and verdict != "FAIL":
            verdict = res
        moved = f"moved {got - frm:+d} of {goal - frm:+d} steps" if None not in (got, frm) else "no reading"
        ld_txt = f"load {ld / 10:+.0f}%" if ld is not None else "load ?"
        print(f"ID {sid}: {label} goal={goal} got={got} ({moved}, {ld_txt}) {res}")
    if verdict == "STALLED":
        print(f"ID {sid}: stalled under load. Likely gravity or a hard stop; the 1:{GEAR_RATIO} motors "
              "have less holding torque than the 1:345 ones the SO-101 follower is designed for.")
    return verdict


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="default: auto-detect the adapter")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", type=int, nargs="+", default=list(JOINTS),
                    help="default: the 6 joint IDs 1-6")
    ap.add_argument("--deg", type=float, default=1.0, help="nudge size; negative = other direction")
    ap.add_argument("--speed", type=int, default=200, help="steps/s")
    ap.add_argument("--max-volts", type=float, default=8.4)  # STS3215 7.4 V version
    args = ap.parse_args()

    args.port = args.port or find_port()
    print(f"Using {args.port}  (STS3215 1:{GEAR_RATIO}, {STEPS_PER_DEG:.2f} steps/deg at the joint)")
    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        ids = [sid for sid in args.ids if transact(ser, sid, PING)]
        silent = [sid for sid in args.ids if sid not in ids]
        if silent:
            print("Not responding:", ", ".join(f"{s} ({JOINTS.get(s, '?')})" for s in silent))
        if not ids:
            print("No servos responded.")
            return
        volts = {sid: v[0] / 10 for sid in ids if (v := read(ser, sid, ADDR_VOLTAGE, 1))}
        if any(v > args.max_volts for v in volts.values()):
            print(f"ABORT: bus voltage {max(volts.values())} V exceeds {args.max_volts} V. "
                  "Check the servo power supply.")
            return

        results = {}
        for sid in ids:
            name = JOINTS.get(sid, "misnumbered, not a joint ID")
            input(f"\nPress Enter to nudge ID {sid} ({name}) by {args.deg} deg (Ctrl+C to stop)...")
            results[sid] = test_servo(ser, sid, args.deg, args.speed)

        print("\nSummary:", "  ".join(f"ID {s} {JOINTS.get(s, '?')}: {r}" for s, r in results.items()))


if __name__ == "__main__":
    main()
