"""Minimal Feetech STS/ST3215 protocol over pyserial (no LeRobot needed).

Register addresses and encodings match LeRobot's Feetech STS table, so
anything written here is what LeRobot itself would write.
"""
import serial

DEFAULT_BAUD = 1_000_000
# Baud_Rate register value -> bits per second
BAUD_CODES = {0: 1_000_000, 1: 500_000, 2: 250_000, 3: 128_000,
              4: 115_200, 5: 57_600, 6: 38_400, 7: 19_200}

# This arm's motor IDs, numbered from the gripper (1) to the base (6).
# Joint names are LeRobot's, but the ID order is the REVERSE of LeRobot's
# so101_follower default (shoulder_pan=1 ... gripper=6). Keep in sync with
# scan_servos.py.
JOINTS = {1: "gripper", 2: "wrist_roll", 3: "wrist_flex",
          4: "elbow_flex", 5: "shoulder_lift", 6: "shoulder_pan"}

# (address, size in bytes)
REG = {
    "Model_Number": (3, 2),
    "ID": (5, 1),
    "Baud_Rate": (6, 1),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "Homing_Offset": (31, 2),   # sign-magnitude, sign bit 11
    "Operating_Mode": (33, 1),  # 0 = position mode
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),    # x100 steps/s^2, 0 = no ramp
    "Goal_Position": (42, 2),
    "Goal_Time": (44, 2),
    "Goal_Speed": (46, 2),      # steps/s
    "Lock": (55, 1),            # 0 = EEPROM writes persist
    "Present_Position": (56, 2),
    "Present_Speed": (58, 2),
    "Present_Load": (60, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
}
# Sign-magnitude encodings (same as LeRobot). With a homing offset applied,
# positions can read below 0 or above 4095, so they must be decoded as signed.
SIGN_BIT = {"Homing_Offset": 11, "Goal_Position": 15, "Present_Position": 15,
            "Present_Speed": 15, "Present_Load": 10}
MAX_RES = 4095  # 12-bit encoder: 0..4095 per output-shaft turn

PING, READ, WRITE = 0x01, 0x02, 0x03


class BusError(RuntimeError):
    pass


def _checksum(body):
    return (~sum(body)) & 0xFF


def _encode(name, value):
    bit = SIGN_BIT.get(name)
    if bit is not None and value < 0:
        value = (-value) | (1 << bit)
    return value


def _decode(name, value):
    bit = SIGN_BIT.get(name)
    if bit is not None and value & (1 << bit):
        value = -(value & ~(1 << bit))
    return value


encode, decode = _encode, _decode  # public names for other modules


class Bus:
    def __init__(self, port, baud=DEFAULT_BAUD, timeout=0.03):
        self.ser = serial.Serial(port, baud, timeout=timeout)

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def set_baud(self, baud):
        self.ser.baudrate = baud

    def _transact(self, sid, instr, params=b""):
        body = bytes([sid, len(params) + 2, instr]) + bytes(params)
        self.ser.reset_input_buffer()
        self.ser.write(b"\xff\xff" + body + bytes([_checksum(body)]))
        head = self.ser.read(4)  # FF FF ID LEN
        if len(head) < 4 or head[:2] != b"\xff\xff":
            return None
        rest = self.ser.read(head[3])  # ERR, data..., CHK
        if len(rest) != head[3] or _checksum(head[2:] + rest[:-1]) != rest[-1]:
            return None
        return rest[1:-1]

    def ping(self, sid, retries=2):
        return any(self._transact(sid, PING) is not None for _ in range(retries))

    def scan(self, ids=range(254)):
        return [sid for sid in ids if self.ping(sid)]

    def read(self, sid, name, retries=3):
        addr, size = REG[name]
        for _ in range(retries):
            data = self._transact(sid, READ, [addr, size])
            if data is not None and len(data) == size:
                raw = data[0] if size == 1 else data[0] | (data[1] << 8)
                return _decode(name, raw)
        raise BusError(f"read {name} from ID {sid} failed")

    def write(self, sid, name, value, retries=3):
        addr, size = REG[name]
        v = _encode(name, int(value))
        payload = [addr, v & 0xFF] if size == 1 else [addr, v & 0xFF, (v >> 8) & 0xFF]
        for _ in range(retries):
            if self._transact(sid, WRITE, payload) is not None:
                return
        raise BusError(f"write {name}={value} to ID {sid} got no reply")

    def read_raw(self, sid, addr, n, retries=3):
        """Read n consecutive register bytes starting at addr."""
        for _ in range(retries):
            data = self._transact(sid, READ, [addr, n])
            if data is not None and len(data) == n:
                return data
        raise BusError(f"block read {addr}+{n} from ID {sid} failed")

    def write_raw(self, sid, addr, data, retries=3):
        """Write consecutive register bytes starting at addr."""
        for _ in range(retries):
            if self._transact(sid, WRITE, [addr, *data]) is not None:
                return
        raise BusError(f"block write at {addr} to ID {sid} got no reply")

    def torque_off_unlock(self, sid):
        """What LeRobot does before touching EEPROM: torque off, lock off."""
        self.write(sid, "Torque_Enable", 0)
        self.write(sid, "Lock", 0)


def find_single_motor(bus):
    """Search every baud rate for exactly one connected motor -> (baud, id)."""
    hits = []
    for baud in BAUD_CODES.values():
        bus.set_baud(baud)
        hits += [(baud, sid) for sid in bus.scan()]
    bus.set_baud(DEFAULT_BAUD)
    return hits
