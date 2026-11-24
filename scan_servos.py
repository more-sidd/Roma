"""Read-only scan of the SO-101 arm's Feetech STS3215 servos on the Waveshare bus adapter.

The SO-101 has exactly 6 motors, IDs 1-6 (see JOINTS). The scan still pings
IDs 0-253 so a misnumbered motor shows up, and flags any ID outside 1-6.
Reads position, voltage, temperature and load. Never writes.
Usage: python scan_servos.py [PORT|auto] [BAUD]   (port auto-detected by default)
"""
import sys
import serial
from serial.tools import list_ports

PING, READ = 0x01, 0x02
ADDR_POSITION, ADDR_LOAD, ADDR_VOLTAGE, ADDR_TEMP = 56, 60, 62, 63

# SO-101: 6 motors, one per joint, numbered from the gripper (1) to the base (6).
# Joint names are LeRobot's, but the ID order is the REVERSE of LeRobot's
# so101_follower default (shoulder_pan=1 ... gripper=6). Keep in sync with sts_bus.py.
JOINTS = {1: "gripper", 2: "wrist_roll", 3: "wrist_flex",
          4: "elbow_flex", 5: "shoulder_lift", 6: "shoulder_pan"}


def find_port():
    """Return the COM port of the Waveshare adapter (WCH CH34x USB-serial chip)."""
    for p in list_ports.comports():
        if p.vid == 0x1A86 or "CH34" in (p.description or ""):
            return p.device
    raise SystemExit("Servo adapter not found. Is the USB cable plugged in? Ports: "
                     + ", ".join(f"{p.device} ({p.description})" for p in list_ports.comports()))


def packet(sid, instr, params=b""):
    body = bytes([sid, len(params) + 2, instr]) + params
    return b"\xff\xff" + body + bytes([~sum(body) & 0xFF])


def transact(ser, sid, instr, params=b""):
    ser.reset_input_buffer()
    ser.write(packet(sid, instr, params))
    head = ser.read(4)
    if len(head) < 4 or head[:2] != b"\xff\xff" or head[2] != sid:
        return None
    rest = ser.read(head[3])
    if len(rest) < head[3] or (~sum(head[2:] + rest[:-1]) & 0xFF) != rest[-1]:
        return None
    return rest[0], rest[1:-1]  # (error byte, data)


def read(ser, sid, addr, n):
    r = transact(ser, sid, READ, bytes([addr, n]))
    return r[1] if r else None


def signed(raw, bit):
    return -(raw & ~(1 << bit)) if raw & (1 << bit) else raw


def main():
    PORT = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] != "auto" else find_port()
    BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 1_000_000
    with serial.Serial(PORT, BAUD, timeout=0.02) as ser:
        found = [sid for sid in range(0, 254) if transact(ser, sid, PING)]
        print(f"{PORT} @ {BAUD}: found {len(found)} servo(s): {found}")
        for sid in found:
            pos, load = read(ser, sid, ADDR_POSITION, 2), read(ser, sid, ADDR_LOAD, 2)
            volt, temp = read(ser, sid, ADDR_VOLTAGE, 1), read(ser, sid, ADDR_TEMP, 1)
            p = signed(int.from_bytes(pos, "little"), 15) if pos else None
            l = signed(int.from_bytes(load, "little"), 10) if load else None
            deg = f"{p * 360 / 4096:6.1f} deg" if p is not None else "?"
            name = JOINTS.get(sid, "NOT AN SO-101 ID (misnumbered motor)")
            print(f"  ID {sid:3} {name:<14}: pos={p} ({deg})  load={l}  "
                  f"{volt[0] / 10 if volt else '?'} V  {temp[0] if temp else '?'} C")

        missing = [f"{i} ({JOINTS[i]})" for i in JOINTS if i not in found]
        extra = [sid for sid in found if sid not in JOINTS]
        if missing:
            print("Missing joints:", ", ".join(missing))
        if extra:
            print(f"IDs outside 1-6: {extra}. The arm has only 6 motors, so each of these is a "
                  "joint with the wrong ID (renumber it), not an extra motor.")
        if not missing and not extra:
            print("All 6 joints present with correct IDs.")


if __name__ == "__main__":
    main()
