"""Read-only scan of the SO-101 servo bus. Never writes to the motors.

Usage:  python so101_scan.py COM16            (one shot)
        python so101_scan.py COM16 --watch    (live joint readout, Ctrl+C to stop)
"""
import sys
import time

from sts_bus import JOINTS, Bus, BusError


def describe(bus, sid):
    try:
        r = {n: bus.read(sid, n) for n in ("Model_Number", "Present_Voltage", "Present_Temperature",
                                           "Torque_Enable", "Homing_Offset", "Min_Position_Limit",
                                           "Max_Position_Limit", "Present_Position")}
    except BusError as e:
        return f"ID {sid}: {e}"
    p = r["Present_Position"]
    return (f"ID {sid} ({JOINTS.get(sid, '?')}): model={r['Model_Number']} "
            f"V={r['Present_Voltage'] / 10:.1f} T={r['Present_Temperature']}C "
            f"torque={'ON' if r['Torque_Enable'] else 'off'} homing_offset={r['Homing_Offset']} "
            f"limits=[{r['Min_Position_Limit']},{r['Max_Position_Limit']}] "
            f"pos={p} ({p * 360 / 4096:.1f} deg)")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    with Bus(sys.argv[1]) as bus:
        ids = bus.scan(range(21))
        if not ids:
            print("No servos answered. Is the servo power supply plugged into the driver board?")
            return
        print("Servos found:", ids)
        missing = [i for i in JOINTS if i not in ids]
        if missing:
            print("Missing joint IDs:", missing)
        for sid in ids:
            print(" ", describe(bus, sid))
        if "--watch" in sys.argv:
            print("\nLive positions (0-4095, ~2047 = calibrated middle). Move the arm by hand. Ctrl+C to stop.")
            try:
                while True:
                    vals = []
                    for sid in ids:
                        try:
                            v = f"{bus.read(sid, 'Present_Position'):>4}"
                        except BusError:
                            v = "----"
                        vals.append(f"{JOINTS.get(sid, sid)}={v}")
                    print("  ".join(vals), end="\r", flush=True)
                    time.sleep(0.05)
            except KeyboardInterrupt:
                print()


if __name__ == "__main__":
    main()
