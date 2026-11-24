#!/usr/bin/env python3
"""
SO-101 stall diagnostic for Feetech STS3215 servos.

Puts the arm in poses you choose, holds each one with torque on, and logs
load, voltage, current, temperature, position error and error flags for
every motor. At the end it prints a per-motor summary with likely causes.

Requires: pip install feetech-servo-sdk   (already installed with lerobot)
Close any LeRobot teleop/calibration process first; only one program can
own the serial port.

Usage:
    python so101_diagnose.py --port /dev/ttyACM0
    python so101_diagnose.py --port /dev/ttyACM0 --hold 30 --csv log.csv
"""

import argparse
import csv
import time

from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# Roma's ID order is gripper -> base (reverse of stock LeRobot). See AGENTS.md.
MOTORS = {
    1: "gripper",
    2: "wrist_roll",
    3: "wrist_flex",
    4: "elbow_flex",
    5: "shoulder_lift",
    6: "shoulder_pan",
}

# STS3215 control table addresses
ADDR_MIN_POS_LIMIT = 9       # EEPROM, 2 bytes
ADDR_MAX_POS_LIMIT = 11      # EEPROM, 2 bytes
ADDR_HOMING_OFFSET = 31      # EEPROM, 2 bytes, bit 11 = sign
ADDR_MAX_TORQUE_LIMIT = 16   # EEPROM, 2 bytes, 0-1000 (0.1 %)
ADDR_TORQUE_ENABLE = 40      # 1 byte
ADDR_GOAL_POSITION = 42      # 2 bytes
ADDR_TORQUE_LIMIT = 48       # RAM, 2 bytes, 0-1000 (0.1 %)
ADDR_PRESENT_POSITION = 56   # 2 bytes, 0-4095
ADDR_PRESENT_LOAD = 60       # 2 bytes, bit 10 = direction, 0-1000 (0.1 %)
ADDR_PRESENT_VOLTAGE = 62    # 1 byte, 0.1 V
ADDR_PRESENT_TEMP = 63       # 1 byte, deg C
ADDR_STATUS = 65             # 1 byte, error flags
ADDR_PRESENT_CURRENT = 69    # 2 bytes, ~6.5 mA per unit

STATUS_BITS = {
    0: "voltage",
    1: "encoder",
    2: "overtemp",
    3: "overcurrent",
    5: "overload",
}


class Bus:
    def __init__(self, port, baud):
        self.port = PortHandler(port)
        self.pkt = PacketHandler(0)  # protocol 0 = STS series
        if not self.port.openPort():
            raise SystemExit(f"Could not open {port}")
        if not self.port.setBaudRate(baud):
            raise SystemExit(f"Could not set baud {baud}")

    def r1(self, mid, addr):
        v, c, _ = self.pkt.read1ByteTxRx(self.port, mid, addr)
        return v if c == COMM_SUCCESS else None

    def r2(self, mid, addr):
        v, c, _ = self.pkt.read2ByteTxRx(self.port, mid, addr)
        return v if c == COMM_SUCCESS else None

    def w1(self, mid, addr, val):
        self.pkt.write1ByteTxRx(self.port, mid, addr, val)

    def w2(self, mid, addr, val):
        self.pkt.write2ByteTxRx(self.port, mid, addr, val)

    def close(self):
        self.port.closePort()


def signed(raw, sign_bit):
    if raw is None:
        return None
    mag = raw & ((1 << sign_bit) - 1)
    return -mag if raw & (1 << sign_bit) else mag


def decode_status(s):
    if not s:
        return ""
    return ",".join(name for bit, name in STATUS_BITS.items() if s & (1 << bit))


def read_pos(bus, mid):
    return signed(bus.r2(mid, ADDR_PRESENT_POSITION), 15)


def read_all(bus, mid):
    load = signed(bus.r2(mid, ADDR_PRESENT_LOAD), 10)
    volt = bus.r1(mid, ADDR_PRESENT_VOLTAGE)
    cur = signed(bus.r2(mid, ADDR_PRESENT_CURRENT), 15)
    return {
        "pos": read_pos(bus, mid),
        "load_pct": None if load is None else load / 10.0,
        "voltage_v": None if volt is None else volt / 10.0,
        "current_ma": None if cur is None else cur * 6.5,
        "temp_c": bus.r1(mid, ADDR_PRESENT_TEMP),
        "status": bus.r1(mid, ADDR_STATUS),
    }


def check_config(bus):
    print("\n=== Motor check ===")
    alive = {}
    for mid, name in MOTORS.items():
        pos = read_pos(bus, mid)
        if pos is None:
            print(f"  ID {mid} {name:14s} NOT RESPONDING")
            continue
        lo = bus.r2(mid, ADDR_MIN_POS_LIMIT)
        hi = bus.r2(mid, ADDR_MAX_POS_LIMIT)
        off = signed(bus.r2(mid, ADDR_HOMING_OFFSET), 11)
        alive[mid] = (lo, hi)
        rng = "  <-- POSITION OUTSIDE LIMITS (calibration/wrap problem)" \
            if lo is not None and hi is not None and not (lo <= pos <= hi) else ""
        print(f"  ID {mid} {name:14s} pos={pos:5d}  limits=[{lo},{hi}]  "
              f"homing_offset={off}{rng}")
        max_tl = bus.r2(mid, ADDR_MAX_TORQUE_LIMIT)
        tl = bus.r2(mid, ADDR_TORQUE_LIMIT)
        volt = bus.r1(mid, ADDR_PRESENT_VOLTAGE)
        flag = ""
        if (max_tl is not None and max_tl < 1000) or (tl is not None and tl < 1000):
            flag = "  <-- torque limit below 100%"
        print(f"  {'':19s}max_torque_limit={max_tl}  "
              f"torque_limit={tl}  V={volt/10 if volt else '?'}{flag}")
    return alive


def hold_test(bus, alive, label, hold_s, rate_hz, writer, last_pose):
    for mid in alive:
        bus.w1(mid, ADDR_TORQUE_ENABLE, 0)
    input(f"\n[{label}] Torque OFF. Move the arm to this pose by hand, "
          f"then hold it there and press Enter...")

    goals = {}
    for mid, (lo, hi) in alive.items():
        p = read_pos(bus, mid)
        goals[mid] = p
        if p is None or p < 0 or (lo is not None and hi is not None and not (lo <= p <= hi)):
            print(f"  SKIP {MOTORS[mid]}: pos {p} outside [{lo},{hi}], torque left OFF "
                  f"so it can't jump")
            continue
        bus.w2(mid, ADDR_GOAL_POSITION, p)
        bus.w1(mid, ADDR_TORQUE_ENABLE, 1)

    if last_pose:
        moved = [MOTORS[m] for m in alive
                 if goals[m] is not None and last_pose.get(m) is not None
                 and abs(goals[m] - last_pose[m]) > 100]
        if not moved:
            print("  WARNING: no joint moved more than ~9 deg since the last pose. "
                  "Did you reposition the arm?")
    print(f"  Torque ON. Let go. Logging for {hold_s}s...")

    stats = {mid: {"load": [], "volt": [], "temp": [], "err": [], "flags": set()}
             for mid in alive}
    t0 = time.time()
    while time.time() - t0 < hold_s:
        t = time.time() - t0
        for mid in alive:
            d = read_all(bus, mid)
            err = None if d["pos"] is None else d["pos"] - goals[mid]
            s = stats[mid]
            if d["load_pct"] is not None:
                s["load"].append(abs(d["load_pct"]))
            if d["voltage_v"] is not None:
                s["volt"].append(d["voltage_v"])
            if d["temp_c"] is not None:
                s["temp"].append(d["temp_c"])
            if err is not None:
                s["err"].append(abs(err))
            flags = decode_status(d["status"])
            if flags:
                s["flags"].update(flags.split(","))
            if writer:
                writer.writerow([label, round(t, 2), mid, MOTORS[mid], d["pos"],
                                 goals[mid], err, d["load_pct"], d["voltage_v"],
                                 d["current_ma"], d["temp_c"], flags])
        time.sleep(1.0 / rate_hz)

    summarize(label, stats)
    return goals


def summarize(label, stats):
    print(f"\n=== Results: {label} ===")
    print(f"  {'joint':14s} {'load max%':>9s} {'load avg%':>9s} {'V min':>6s} "
          f"{'temp rise':>9s} {'pos err max':>11s}  notes")
    for mid, s in stats.items():
        if not s["load"]:
            continue
        lmax, lavg = max(s["load"]), sum(s["load"]) / len(s["load"])
        vmin = min(s["volt"]) if s["volt"] else 0
        vmax = max(s["volt"]) if s["volt"] else 0
        trise = (s["temp"][-1] - s["temp"][0]) if len(s["temp"]) > 1 else 0
        emax = max(s["err"]) if s["err"] else 0

        notes = []
        if lmax >= 90:
            notes.append("at torque limit -> undersized/gravity")
        elif lavg >= 50:
            notes.append("high continuous load")
        if emax > 40 and lmax < 60:
            notes.append("sagging at low load -> check torque limit/PID")
        elif emax > 40:
            notes.append("sagging")
        if vmin and vmin < 6.5:
            notes.append("low voltage -> supply/wiring (7.4V version)")
        if vmax - vmin >= 0.8:
            notes.append(f"supply sags {vmax - vmin:.1f} V under load")
        if trise >= 5:
            notes.append("heating fast")
        if s["flags"]:
            notes.append("FLAGS: " + ",".join(sorted(s["flags"])))

        print(f"  {MOTORS[mid]:14s} {lmax:9.1f} {lavg:9.1f} {vmin:6.1f} "
              f"{trise:9d} {emax:11d}  {'; '.join(notes)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--hold", type=int, default=20, help="seconds per pose")
    ap.add_argument("--rate", type=float, default=10, help="samples/s per motor")
    ap.add_argument("--csv", default="so101_diag.csv")
    args = ap.parse_args()

    bus = Bus(args.port, args.baud)
    f = open(args.csv, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["pose", "t_s", "id", "joint", "pos", "goal", "pos_err",
                     "load_pct", "voltage_v", "current_ma", "temp_c", "flags"])

    try:
        alive = check_config(bus)
        if not alive:
            raise SystemExit("No motors responded. Check port, power and IDs.")

        poses = [
            "1 folded / rest (low gravity load)",
            "2 upper arm vertical, forearm horizontal",
            "3 whole arm stretched out horizontally (worst case)",
        ]
        last = None
        for p in poses:
            last = hold_test(bus, alive, p, args.hold, args.rate, writer, last)
            f.flush()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        print("\nDisabling torque. Support the arm!")
        time.sleep(1.0)
        for mid in MOTORS:
            bus.w1(mid, ADDR_TORQUE_ENABLE, 0)
        bus.close()
        f.close()
        print(f"Log saved to {args.csv}")


if __name__ == "__main__":
    main()
