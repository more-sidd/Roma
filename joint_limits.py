"""Show and measure the min/max limits of each SO-101 joint. Read-only, never writes.

Prints three things per joint:
  1. Design limits from the official SO-101 URDF (TheRobotStudio/SO-ARM100,
     Simulation/SO101/so101_new_calib.urdf), in degrees from the calibrated middle.
  2. The limit window currently stored in the motor (Min/Max_Position_Limit, regs 9/11).
  3. With --measure: the actual min/max you reach moving each joint by hand
     (torque must be off), in steps and degrees, so you can compare to (1).

Steps -> degrees is 4096 steps per 360 deg at the joint. That holds for the
1:147 motors on this arm because the encoder is on the output shaft.

Usage: python joint_limits.py [--port COMx] [--measure]
"""
import argparse
import math
import threading
import time

import serial

from scan_servos import ADDR_POSITION, JOINTS, PING, find_port, read, signed, transact

STEPS_PER_DEG = 4096 / 360

# radians, straight from so101_new_calib.urdf (0 rad = calibrated middle of range)
URDF_LIMITS_RAD = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}


def u16(ser, sid, addr):
    for _ in range(3):
        if (raw := read(ser, sid, addr, 2)) is not None:
            return int.from_bytes(raw, "little")
    return None


def pos(ser, sid):
    raw = u16(ser, sid, ADDR_POSITION)
    return signed(raw, 15) if raw is not None else None


def measure(ser, ids):
    lo = {i: pos(ser, i) for i in ids}
    hi, now = dict(lo), dict(lo)
    done = threading.Event()
    threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()
    print(" | ".join(f"{JOINTS.get(i, i):^16}" for i in ids))
    while not done.is_set():
        for i in ids:
            if (p := pos(ser, i)) is not None:
                now[i], lo[i], hi[i] = p, min(lo[i], p), max(hi[i], p)
        print(" | ".join(f"{lo[i]:>5}{now[i]:>5}{hi[i]:>5} " for i in ids), end="\r", flush=True)
        time.sleep(0.02)
    print()
    return lo, hi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="default: auto-detect the adapter")
    ap.add_argument("--measure", action="store_true", help="record min/max while you move each joint")
    args = ap.parse_args()

    port = args.port or find_port()
    with serial.Serial(port, 1_000_000, timeout=0.02) as ser:
        ids = [i for i in JOINTS if transact(ser, i, PING)]
        print(f"{port}: joints answering {ids}\n")

        print(f"{'joint':<14} {'URDF min':>9} {'URDF max':>9} {'travel':>7}   motor limit window")
        for i in JOINTS:
            name = JOINTS[i]
            a, b = (math.degrees(x) for x in URDF_LIMITS_RAD[name])
            win = "no reply"
            if i in ids:
                lo, hi = u16(ser, i, 9), u16(ser, i, 11)
                win = f"[{lo}, {hi}]" + ("  (factory default, not calibrated)" if (lo, hi) == (0, 4095) else "")
            print(f"{name:<14} {a:>8.1f}° {b:>8.1f}° {b - a:>6.0f}°   {win}")

        if not args.measure:
            print("\nRun with --measure to record the real range of each joint by hand.")
            return

        if any(read(ser, i, 40, 1) not in (None, b"\x00") for i in ids):
            raise SystemExit("\nTorque is ON for some joint. Power-cycle the servo supply so the arm is limp.")
        print("\nTorque is off. Move EACH joint slowly to BOTH of its ends. Columns: min now max.")
        print("Press ENTER when done.\n")
        lo, hi = measure(ser, ids)

        print(f"\n{'joint':<14} {'min':>6} {'max':>6} {'span':>6} {'measured':>9} {'URDF':>6}")
        for i in ids:
            name = JOINTS[i]
            span = hi[i] - lo[i]
            urdf = math.degrees(URDF_LIMITS_RAD[name][1] - URDF_LIMITS_RAD[name][0])
            print(f"{name:<14} {lo[i]:>6} {hi[i]:>6} {span:>6} {span / STEPS_PER_DEG:>8.0f}° {urdf:>5.0f}°")
        print("\nmin/max are in motor steps and still include the old homing offsets, so only the span")
        print("is meaningful until you run so101_calibrate.py, which re-centres each joint at ~2047")
        print("and stores these min/max as the motor's limit window.")


if __name__ == "__main__":
    main()
