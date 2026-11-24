"""Forward kinematics and damped-least-squares IK for the SO-101 arm (pure Python).

Geometry is copied from the official SO-101 URDF (TheRobotStudio/SO-ARM100,
Simulation/SO101/so101_new_calib.urdf): per joint, T = Trans(xyz) * Rz(y) Ry(p) Rx(r) * Rz(q).
"Calibrated zero" (the middle of each joint's range, motor reading 2047) is q = 0 in that
URDF, so q = (position - 2047) / steps_per_deg only means something after calibration.

Frame: URDF base_link (z up). Tool point = the URDF's gripper_frame_link origin.
Task space is [x, y, z, pitch] with pitch = elevation of the tool's z axis (the gripper's
pointing direction) above the horizontal, in radians. The IK moves the joints you pass as
`free`; every other joint keeps the angle you give it.
"""
import math

# (xyz, rpy) per joint, base -> wrist roll, then the fixed tool frame. Order = JOINT_ORDER.
JOINT_ORDER = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
_CHAIN = (
    ((0.0388353, -8.97657e-09, 0.0624), (3.14159, 4.18253e-17, -3.14159)),
    ((-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0.0)),
    ((-0.11257, -0.028, 1.73763e-16), (-3.63608e-16, 8.74301e-16, 1.5708)),
    ((-0.1349, 0.0052, 3.62355e-17), (4.02456e-15, 8.67362e-16, -1.5708)),
    ((5.55112e-17, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159)),
)
_TOOL = ((-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0))
# URDF joint limits (rad), same order
LIMITS = ((-1.91986, 1.91986), (-1.74533, 1.74533), (-1.69, 1.69), (-1.65806, 1.65806), (-2.74385, 2.84121))


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _origin(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
            [-sp, cp * sr, cp * cr, xyz[2]],
            [0.0, 0.0, 0.0, 1.0]]


def _rotz(q):
    c, s = math.cos(q), math.sin(q)
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


_ORIGINS = [_origin(*c) for c in _CHAIN]
_TOOL_T = _origin(*_TOOL)


def fk(q):
    """q: 5 joint angles in radians (JOINT_ORDER). Returns (x, y, z, pitch)."""
    t = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    for o, qi in zip(_ORIGINS, q):
        t = _mul(_mul(t, o), _rotz(qi))
    t = _mul(t, _TOOL_T)
    dx, dy, dz = t[0][2], t[1][2], t[2][2]  # tool z axis in the base frame
    return (t[0][3], t[1][3], t[2][3], math.atan2(dz, math.hypot(dx, dy)))


def _solve(a, b):
    """Solve a x = b (small dense, Gaussian elimination with pivoting)."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            for k in range(c, n + 1):
                m[r][k] -= f * m[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


def ik_step(q, target, free, lam=0.02, pitch_weight=0.05, max_dq=0.1, limits=LIMITS):
    """One damped-least-squares step towards target = (x, y, z, pitch).

    q: 5 angles (rad). free: indices into JOINT_ORDER that may move. pitch_weight scales the
    pitch error (rad) against position error (m) so position wins when they conflict.
    Each moved joint is limited to max_dq rad per step and to `limits`. Returns (new_q,
    error_vector_before) with error as (dx, dy, dz, dpitch) in m / rad.
    """
    cur = fk(q)
    err = [target[i] - cur[i] for i in range(3)] + [_wrap(target[3] - cur[3])]
    if not free:
        return list(q), err
    eps = 1e-5
    cols = []
    for j in free:
        q2 = list(q)
        q2[j] += eps
        f = fk(q2)
        cols.append([(f[0] - cur[0]) / eps, (f[1] - cur[1]) / eps, (f[2] - cur[2]) / eps,
                     _wrap(f[3] - cur[3]) / eps * pitch_weight])  # one column per free joint
    e = err[:3] + [err[3] * pitch_weight]
    jjt = [[sum(cols[k][a] * cols[k][b] for k in range(len(free))) + (lam * lam if a == b else 0.0)
            for b in range(4)] for a in range(4)]
    y = _solve(jjt, e)
    dq = [sum(cols[k][a] * y[a] for a in range(4)) for k in range(len(free))]
    big = max((abs(d) for d in dq), default=0.0)
    if big > max_dq:
        dq = [d * max_dq / big for d in dq]
    new = list(q)
    for k, j in enumerate(free):
        lo, hi = limits[j]
        new[j] = max(lo, min(hi, q[j] + dq[k]))
    return new, err


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def solve(q, target, free, iters=20, tol=1e-4, **kw):
    """Iterate ik_step. Returns (q, residual_position_error_m)."""
    for _ in range(iters):
        q, err = ik_step(q, target, free, **kw)
        if math.sqrt(sum(e * e for e in err[:3])) < tol and abs(err[3]) < 1e-3:
            break
    f = fk(q)
    return q, math.sqrt(sum((target[i] - f[i]) ** 2 for i in range(3)))
