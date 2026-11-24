"""Read-only check of the motor-direction assumption (DIR in remote.py) for Cartesian control.

Needs arm_ui.py running on the real arm (it owns the serial port) and torque OFF.
Just move ONE joint by hand, in any direction, then let go and hold still for a second.
The script then says what it computes the gripper did (UP/DOWN, LEFT/RIGHT, pitch UP/DOWN)
and you compare that with what really happened. If they disagree, that joint's DIR is -1.
Ctrl+C to stop. Run: python dir_check.py
"""
import json
import math
import time
import urllib.request

import kinematics as kin

URL = "http://127.0.0.1:8101/api/state"
SPD = 4096 / 360
ORDER = (6, 5, 4, 3, 2)
NAMES = {1: "gripper", 2: "wrist_roll", 3: "wrist_flex", 4: "elbow_flex", 5: "shoulder_lift", 6: "shoulder_pan"}


def state():
    s = json.load(urllib.request.urlopen(URL, timeout=3))
    if s["sim"]:
        raise SystemExit("That server is the SIMULATOR. Run the real one.")
    return {m["id"]: m["pos"] for m in s["motors"]}, any(m["torque"] for m in s["motors"])


def tool(p):
    return kin.fk([(p[i] - 2047) / SPD * math.pi / 180 for i in ORDER])


def words(a, b, moved):
    d = [b[k] - a[k] for k in range(4)]
    if moved == 6:
        return f"gripper moved {'LEFT' if d[1] > 0 else 'RIGHT'} by {abs(d[1])*1000:.0f} mm (seen from behind the arm)"
    if moved in (5, 4):
        return (f"gripper moved {'UP' if d[2] > 0 else 'DOWN'} by {abs(d[2])*1000:.0f} mm and "
                f"{'FORWARD' if d[0] > 0 else 'BACK'} by {abs(d[0])*1000:.0f} mm")
    if moved == 3:
        return f"gripper tip tilted {'UP' if d[3] > 0 else 'DOWN'} by {abs(math.degrees(d[3])):.0f} deg"
    return "(not used by the IK)"


p, torque = state()
if torque:
    raise SystemExit("Some joints have torque on. Press 'Torque off all' in the UI first.")
print("Move ONE joint by hand, let go, hold still. Repeat for the base, shoulder, elbow, wrist flex. Ctrl+C to stop.\n")
base, last, still_since = p, p, time.time()
while True:
    time.sleep(0.1)
    p, _ = state()
    if max(abs(p[i] - last[i]) for i in p) > 3:
        last, still_since = p, time.time()
        continue
    if time.time() - still_since > 1.0:
        delta = {i: p[i] - base[i] for i in p}
        moved = max(delta, key=lambda i: abs(delta[i]))
        others = max((abs(v) for i, v in delta.items() if i != moved), default=0)
        if abs(delta[moved]) > 60:
            if others > 0.4 * abs(delta[moved]):
                print("More than one joint moved. Please move just one at a time.\n")
            else:
                print(f"ID {moved} {NAMES[moved]} moved {delta[moved]:+d} steps ({delta[moved]/SPD:+.0f} deg)")
                print(f"   computed: {words(tool(base), tool(p), moved)}")
                print("   Did the real gripper do that?  yes = DIR +1 is right, no = it should be -1\n")
        base = p
