"""Assign SO-101 motor IDs (1-6) and set baud to 1,000,000, like `lerobot-setup-motors`.

This arm is numbered from the gripper to the base (see JOINTS in sts_bus.py):
  gripper=1, wrist_roll=2, wrist_flex=3, elbow_flex=4, shoulder_lift=5, shoulder_pan=6.
That is the reverse of LeRobot's so101_follower default.

Modes:

  python so101_setup_ids.py COM16 --change OLD NEW
      Renumber one motor on the assembled bus (e.g. --change 7 3).
      NEW must not already be in use.

  python so101_setup_ids.py COM16 --one JOINT
      Connect ONLY that motor to the board (unplug the rest), then run.
      Finds it at any ID/baud and sets the ID for JOINT (name or 1-6).

  python so101_setup_ids.py COM16 --all
      Guided: gripper (ID 1) first up to shoulder_pan (ID 6), one motor connected at a time.

  python so101_setup_ids.py COM16 --identify
      Fully assembled arm, IDs in the wrong order. Torque off; you wiggle one
      joint at a time, gripper to base, and it detects which motor moved.
      It prints the mapping and, only after you type YES, renumbers to the
      gripper=1 ... shoulder_pan=6 order above.
"""
import sys
import threading
import time

from sts_bus import BAUD_CODES, DEFAULT_BAUD, JOINTS, Bus, BusError, find_single_motor

TEMP_ID_BASE = 200  # parking IDs used while swapping, so two motors never share an ID
MIN_WIGGLE = 60     # steps (~5 deg) a joint must move to count as "the one that moved"

NAME_TO_ID = {v: k for k, v in JOINTS.items()}


def joint_id(arg):
    if arg.isdigit() and int(arg) in JOINTS:
        return int(arg)
    if arg in NAME_TO_ID:
        return NAME_TO_ID[arg]
    sys.exit(f"Unknown joint '{arg}'. Use one of: {', '.join(NAME_TO_ID)} or 1-6.")


def set_id_and_baud(bus, cur_baud, cur_id, new_id):
    bus.set_baud(cur_baud)
    bus.torque_off_unlock(cur_id)
    if cur_id != new_id:
        try:  # the motor may reply under its new ID, so don't trust the reply
            bus.write(cur_id, "ID", new_id, retries=1)
        except BusError:
            pass
        if not bus.ping(new_id, retries=5):
            raise BusError(f"motor did not answer as ID {new_id} after the ID write")
    baud_code = next(k for k, v in BAUD_CODES.items() if v == DEFAULT_BAUD)
    if bus.read(new_id, "Baud_Rate") != baud_code:
        bus.write(new_id, "Baud_Rate", baud_code)
    bus.set_baud(DEFAULT_BAUD)
    if not bus.ping(new_id, retries=5):
        raise BusError(f"motor did not answer as ID {new_id} at {DEFAULT_BAUD} baud afterwards")
    print(f"  OK: now ID {new_id} ({JOINTS[new_id]}) at {DEFAULT_BAUD} baud")


def change(bus, old, new):
    if new not in JOINTS:
        sys.exit("NEW must be 1-6.")
    if not bus.ping(old):
        sys.exit(f"No motor answers as ID {old}.")
    if bus.ping(new):
        sys.exit(f"ID {new} is already taken. Free it first or use --one with only one motor connected.")
    print(f"Renumbering ID {old} -> {new} ({JOINTS[new]})")
    set_id_and_baud(bus, DEFAULT_BAUD, old, new)


def rename(bus, old, new):
    """ID-only write at the current baud, verified by pinging the new ID."""
    bus.torque_off_unlock(old)
    try:  # the reply may come back under the new ID, so don't trust it
        bus.write(old, "ID", new, retries=1)
    except BusError:
        pass
    if not bus.ping(new, retries=5):
        raise BusError(f"motor did not answer as ID {new} after renaming from {old}")


def watch_until_enter(bus, ids):
    """Sample positions until ENTER; return how far each ID moved (max - min)."""
    lo = {i: bus.read(i, "Present_Position") for i in ids}
    hi = dict(lo)
    done = threading.Event()
    threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()
    while not done.is_set():
        for i in ids:
            try:
                p = bus.read(i, "Present_Position", retries=1)
            except BusError:
                continue
            lo[i], hi[i] = min(lo[i], p), max(hi[i], p)
        time.sleep(0.01)
    return {i: hi[i] - lo[i] for i in ids}


def identify(bus):
    ids = bus.scan(range(TEMP_ID_BASE))
    if len(ids) != len(JOINTS):
        sys.exit(f"Found {len(ids)} motors {ids}; the SO-101 has exactly 6. Fix wiring first.")
    if any(bus.read(i, "Torque_Enable") for i in ids):
        sys.exit("Some motors have torque ON. Turn it off (power-cycle the servo supply) and rerun.")

    print("Torque is off. For each joint below, wiggle ONLY that joint back and forth a few times\n"
          "(about 20 degrees or more), keep the rest still, then press ENTER.\n")
    found = {}  # target joint ID -> current motor ID
    for tid, name in JOINTS.items():
        while True:
            print(f"[{tid}/6] wiggle {name.upper()} now, then press ENTER ...", end=" ", flush=True)
            moved = watch_until_enter(bus, ids)
            free = sorted((m for m in moved if m not in found.values()), key=moved.get, reverse=True)
            best = free[0]
            runner_up = moved[free[1]] if len(free) > 1 else 0
            if moved[best] >= MIN_WIGGLE and moved[best] >= 3 * max(runner_up, 1):
                found[tid] = best
                print(f"-> motor currently ID {best} (moved {moved[best]} steps)")
                break
            print(f"unclear (biggest movers: {[(m, moved[m]) for m in free[:3]]}). Try again, bigger.")

    print("\nJoint            current ID -> new ID")
    for tid, cur in found.items():
        print(f"  {JOINTS[tid]:<14} {cur:>6}     -> {tid}{'   (unchanged)' if cur == tid else ''}")
    todo = {tid: cur for tid, cur in found.items() if cur != tid}
    if not todo:
        print("\nAll IDs already in the right order. Nothing to write.")
        return
    if input(f"\nWrite {len(todo)} new ID(s) to the motors' EEPROM? Type YES to continue: ").strip() != "YES":
        print("Nothing written.")
        return

    for tid, cur in todo.items():   # step 1: park each motor on a free temp ID
        rename(bus, cur, TEMP_ID_BASE + tid)
    for tid in todo:                # step 2: temp ID -> final ID
        rename(bus, TEMP_ID_BASE + tid, tid)
        print(f"  OK: {JOINTS[tid]} is now ID {tid}")
    final = bus.scan(range(TEMP_ID_BASE + 10))
    print("Bus now:", final, "(expected [1, 2, 3, 4, 5, 6])")


def one(bus, target):
    hits = find_single_motor(bus)
    if not hits:
        sys.exit("No motor found at any baud. Check the cable and the servo power supply.")
    if len(hits) > 1:
        sys.exit(f"More than one motor answered {hits}. Connect only the {JOINTS[target]} motor.")
    baud, sid = hits[0]
    print(f"Found motor: ID {sid} at {baud} baud")
    set_id_and_baud(bus, baud, sid, target)


def main():
    a = sys.argv[1:]
    if len(a) < 2:
        sys.exit(__doc__)
    port, mode = a[0], a[1]
    with Bus(port) as bus:
        if mode == "--change" and len(a) == 4:
            change(bus, int(a[2]), int(a[3]))
        elif mode == "--identify":
            identify(bus)
        elif mode == "--one" and len(a) == 3:
            one(bus, joint_id(a[2]))
        elif mode == "--all":
            for jid in sorted(JOINTS):  # gripper (1) first, base (6) last
                input(f"\nConnect ONLY the '{JOINTS[jid]}' motor to the board, then press ENTER...")
                one(bus, jid)
            print("\nAll six motors set. Reconnect the full chain and run so101_scan.py to check.")
        else:
            sys.exit(__doc__)


if __name__ == "__main__":
    main()
