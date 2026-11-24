"""Calibrate an SO-101 follower the same way `lerobot-calibrate` does, without LeRobot.

  python so101_calibrate.py COM16 [--id my_follower]

Steps (torque stays OFF the whole time, so the arm is limp; support it):
  1. Reset each motor's Homing_Offset to 0 and limits to 0..4095.
  2. You put every joint in the MIDDLE of its range, press ENTER.
     Homing_Offset = position - 2047, so that pose now reads ~2047.
  3. You move every joint through its FULL range, press ENTER.
     The min/max seen become range_min/range_max and are written to the
     motors' position limits.
  4. Saves JSON in LeRobot's format to
     ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<id>.json
     (where LeRobot looks) plus a copy in ./calibration/.

Motor IDs follow this arm's numbering, gripper=1 ... shoulder_pan=6 (JOINTS in
sts_bus.py), and each entry's "id" in the JSON records that. Stock LeRobot
so101_follower expects the reverse (shoulder_pan=1 ... gripper=6), so before
using LeRobot either renumber the motors or give LeRobot this arm's motor map.
"""
import argparse
import json
import os
import threading
import time
from pathlib import Path

from sts_bus import JOINTS, MAX_RES, Bus, BusError

HALF_TURN = MAX_RES // 2  # 2047


def lerobot_calib_dir():
    hf_home = Path(os.environ.get("HF_HOME", Path.home() / ".cache" / "huggingface"))
    base = Path(os.environ.get("HF_LEROBOT_CALIBRATION", hf_home / "lerobot" / "calibration"))
    return base / "robots" / "so101_follower"


def record_ranges(bus):
    mins = {j: bus.read(j, "Present_Position") for j in JOINTS}
    maxs = dict(mins)
    done = threading.Event()
    threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()
    now = dict(mins)
    print(" | ".join(f"{JOINTS[j][:14]:^16}" for j in JOINTS))
    while not done.is_set():
        for j in JOINTS:
            try:
                now[j] = bus.read(j, "Present_Position")
            except BusError:
                continue
            mins[j], maxs[j] = min(mins[j], now[j]), max(maxs[j], now[j])
        print(" | ".join(f"{mins[j]:>4} {now[j]:>4} {maxs[j]:>4}  " for j in JOINTS), end="\r", flush=True)
        time.sleep(0.02)
    print()
    return mins, maxs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("--id", default="my_follower", help="robot id, becomes the JSON file name")
    args = ap.parse_args()

    with Bus(args.port) as bus:
        missing = [f"{j} ({JOINTS[j]})" for j in JOINTS if not bus.ping(j)]
        if missing:
            raise SystemExit(f"Not answering: {', '.join(missing)}. Run so101_setup_ids.py first.")

        print("Torque OFF on all joints (arm goes limp, hold it).")
        for j in JOINTS:
            bus.torque_off_unlock(j)
            bus.write(j, "Operating_Mode", 0)
            bus.write(j, "Homing_Offset", 0)
            bus.write(j, "Min_Position_Limit", 0)
            bus.write(j, "Max_Position_Limit", MAX_RES)

        input("\nSTEP 1: move every joint to the MIDDLE of its range of motion, then press ENTER...")
        offsets = {}
        for j in JOINTS:
            raw = bus.read(j, "Present_Position")
            offsets[j] = raw - HALF_TURN
            bus.write(j, "Homing_Offset", offsets[j])
            print(f"  {JOINTS[j]:<14} raw={raw:>4} -> homing_offset={offsets[j]:>5} -> reads {bus.read(j, 'Present_Position')}")

        print("\nSTEP 2: move EACH joint slowly through its ENTIRE range (both ends).")
        print("Columns per joint: min  now  max. Press ENTER when done.\n")
        mins, maxs = record_ranges(bus)

        bad = [JOINTS[j] for j in JOINTS if maxs[j] - mins[j] < 100]
        if bad:
            raise SystemExit(f"These joints barely moved, calibration NOT saved: {', '.join(bad)}. "
                             "Motor offsets were changed; just run this script again.")

        calib = {}
        for j, name in JOINTS.items():
            bus.write(j, "Min_Position_Limit", mins[j])
            bus.write(j, "Max_Position_Limit", maxs[j])
            calib[name] = {"id": j, "drive_mode": 0, "homing_offset": offsets[j],
                           "range_min": mins[j], "range_max": maxs[j]}

    text = json.dumps(calib, indent=4)
    for d in (lerobot_calib_dir(), Path(__file__).parent / "calibration"):
        d.mkdir(parents=True, exist_ok=True)
        (d / f"{args.id}.json").write_text(text)
        print(f"Saved {d / f'{args.id}.json'}")

    print("\nResult:")
    for name, c in calib.items():
        span = c["range_max"] - c["range_min"]
        print(f"  {name:<14} offset={c['homing_offset']:>5}  range=[{c['range_min']:>4}, {c['range_max']:>4}]"
              f"  ~{span * 360 / 4096:.0f} deg of travel")


if __name__ == "__main__":
    main()
