"""Nudge each STS3215 servo by a small angle, one at a time, then return it.

For each servo: read position -> enable torque -> move +DEG -> move back ->
restore the original torque state. Aborts if bus voltage is out of range.

Usage: python test_servos.py [--port COMx] [--ids 1 2 3] [--deg 1.0] [--max-volts 8.4]
"""
import argparse
import time

import serial

from scan_servos import ADDR_POSITION, ADDR_VOLTAGE, PING, find_port, read, signed, transact

WRITE = 0x03
ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION = 40, 42
STEPS_PER_DEG = 4096 / 360
TOLERANCE = 6  # steps (~0.5 deg); STS3215 settles 3-5 steps short (P-only control, gear friction)


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


def position(ser, sid):
    raw = read_retry(ser, sid, ADDR_POSITION, 2)
    return signed(int.from_bytes(raw, "little"), 15) if raw else None


def move(ser, sid, goal, speed):
    # goal position (2), goal time (2, 0 = use speed), goal speed (2)
    data = goal.to_bytes(2, "little") + (0).to_bytes(2, "little") + speed.to_bytes(2, "little")
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


def test_servo(ser, sid, deg, speed):
    start = position(ser, sid)
    torque = read_retry(ser, sid, ADDR_TORQUE_ENABLE, 1)
    if start is None or torque is None:
        print(f"ID {sid}: no response, skipped")
        return False

    delta = round(deg * STEPS_PER_DEG)
    target = max(0, min(4095, start + delta))
    ok = True
    try:
        move(ser, sid, start, speed)  # hold current pose so enabling torque doesn't jump
        write(ser, sid, ADDR_TORQUE_ENABLE, [1])
        # Judge by measured position, not by whether the write was acknowledged.
        move(ser, sid, target, speed)
        reached = settle(ser, sid, target)
        move(ser, sid, start, speed)
        back = settle(ser, sid, start)
    finally:
        write(ser, sid, ADDR_TORQUE_ENABLE, torque)

    for label, frm, goal, got in (("out ", start, target, reached), ("back", reached, start, back)):
        good = got is not None and abs(got - goal) <= TOLERANCE
        ok &= good
        moved = f"moved {got - frm:+d} of {goal - frm:+d} steps" if None not in (got, frm) else "no reading"
        print(f"ID {sid}: {label} goal={goal} got={got} ({moved}) {'OK' if good else 'FAIL'}")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="default: auto-detect the adapter")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", type=int, nargs="+", default=list(range(1, 8)))
    ap.add_argument("--deg", type=float, default=1.0)
    ap.add_argument("--speed", type=int, default=200, help="steps/s")
    ap.add_argument("--max-volts", type=float, default=8.4)  # STS3215 7.4 V version
    args = ap.parse_args()

    args.port = args.port or find_port()
    print(f"Using {args.port}")
    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        ids = [sid for sid in args.ids if transact(ser, sid, PING)]
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
            input(f"\nPress Enter to nudge ID {sid} by {args.deg} deg (Ctrl+C to stop)...")
            results[sid] = test_servo(ser, sid, args.deg, args.speed)

        print("\nSummary:", "  ".join(f"ID {s}: {'OK' if r else 'FAIL'}" for s, r in results.items()))


if __name__ == "__main__":
    main()
