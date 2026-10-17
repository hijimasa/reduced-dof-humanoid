"""Leg IK for the active robot, on its MJCF with the trunk held at the origin."""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, mujoco

from robot_profile import profile
from joint_names import jn, LEG_ORDER

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MJCF = os.path.join(REPO, profile().MODEL.lstrip('./'))
LEG = {s: [jn(s, b) for b in LEG_ORDER] for s in ('right', 'left')}

_m = mujoco.MjModel.from_xml_path(MJCF)
_d = mujoco.MjData(_m)

# Passive joints slaved to actuated ones by a linkage, e.g. a parallel-link leg whose
# ankle pitch is always the negative of its knee. mj_forward does not solve equality
# constraints, so the model's <connect> loops are no help here: the coupling has to be
# applied to qpos by hand, and folded into the Jacobian so the solver sees that moving
# the knee does NOT tilt the sole. {passive: [(actuated, gain), ...]} from the profile.
COUPLING = {k: list(v) for k, v in dict(getattr(profile(), 'PASSIVE_COUPLING', {})).items()}


def jid(n):
    return mujoco.mj_name2id(_m, mujoco.mjtObj.mjOBJ_JOINT, n)


def qadr(n):
    return _m.jnt_qposadr[jid(n)]


def dadr(n):
    return _m.jnt_dofadr[jid(n)]


def sid(n):
    return mujoco.mj_name2id(_m, mujoco.mjtObj.mjOBJ_SITE, n)


JOINTS = [mujoco.mj_id2name(_m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(_m.njnt)
          if mujoco.mj_id2name(_m, mujoco.mjtObj.mjOBJ_JOINT, i) != 'trunk_freejoint']
LIMITS = {n: np.degrees(_m.jnt_range[jid(n)]) for n in JOINTS}


def _couple():
    for p, terms in COUPLING.items():
        _d.qpos[qadr(p)] = sum(k * _d.qpos[qadr(a)] for a, k in terms)


def _reset(q=None):
    _d.qpos[:] = 0
    _d.qpos[3] = 1.0
    if q:
        for n, v in q.items():
            _d.qpos[qadr(n)] = np.deg2rad(v)
    _couple()
    mujoco.mj_forward(_m, _d)


_reset()
REST = {s: (_d.site_xpos[sid(f'{s}_foot')].copy(),
            _d.site_xmat[sid(f'{s}_foot')].copy().reshape(3, 3)) for s in ('right', 'left')}


def _flex_sign(side):
    """Which way this robot's knee bends: +1 if a positive angle takes the foot back."""
    n = jn(side, 'knee')
    out = {}
    for probe in (-10.0, 10.0):
        _reset({n: probe})
        out[probe] = _d.site_xpos[sid(f'{side}_foot')][0]
    return 1 if out[10.0] < out[-10.0] else -1


FLEX = {s: _flex_sign(s) for s in ('right', 'left')}
_reset()


def rotz(deg):
    c, s = np.cos(np.deg2rad(deg)), np.sin(np.deg2rad(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def solve_leg(side, target_xyz, yaw_deg=0.0, seed=None, iters=200, R_extra=None):
    """See _solve_leg. Retries from a bent-knee seed if the straight-leg start fails.

    A straight leg is the singular configuration between the two IK branches, and with
    the joint clamps in place a step into the unreachable one now stalls against a limit
    instead of quietly finishing hyperextended. Nudging the knee into flexion first picks
    the branch. microban never needs the retry, which is why it is a retry and not the
    default -- its keyframes come out bit-for-bit as before.
    """
    res, pe, re = _solve_leg(side, target_xyz, yaw_deg, seed, iters, R_extra)
    if seed is None and (pe > 1e-5 or re > 1e-4):
        bent = {jn(side, 'knee'): 20.0 * FLEX[side]}
        res, pe, re = _solve_leg(side, target_xyz, yaw_deg, bent, iters, R_extra)
    return res, pe, re


def _solve_leg(side, target_xyz, yaw_deg=0.0, seed=None, iters=200, R_extra=None):
    """Joint angles (deg) putting the <side>_foot sole frame at target_xyz with the
    sole flat and rotated by yaw_deg about z, all expressed in the trunk frame.

    R_extra pre-multiplies the goal orientation: pass R_trunk^T to keep the sole
    horizontal in the *world* while the trunk itself is tilted (used by the get-up)."""
    names = LEG[side]
    q = dict(seed) if seed else {n: 0.0 for n in names}
    _reset(q)
    R_goal = rotz(yaw_deg) @ REST[side][1]
    if R_extra is not None:
        R_goal = R_extra @ R_goal
    p_goal = np.asarray(target_xyz, float)
    dofs = [dadr(n) for n in names]
    # Clamp inside the loop, not after. A leg has two solutions for most targets -- knee
    # forward and knee backward -- and an unconstrained Jacobian step will happily settle
    # in the wrong one: on OP3 the right knee's positive direction IS hyperextension, and
    # the walk it produced was numerically perfect and anatomically impossible. Clamping
    # every iteration keeps the search in the reachable branch, and a target that needs
    # the other one now fails the caller's error check instead of quietly succeeding.
    lim = np.array([np.deg2rad(LIMITS[n]) for n in names])
    jacp, jacr = np.zeros((3, _m.nv)), np.zeros((3, _m.nv))
    for _ in range(iters):
        s = sid(f'{side}_foot')
        p, R = _d.site_xpos[s], _d.site_xmat[s].reshape(3, 3)
        e_p = p_goal - p
        dR = R_goal @ R.T
        w = np.zeros(3)
        mujoco.mju_mat2Quat(qt := np.zeros(4), dR.flatten())
        mujoco.mju_quat2Vel(w, qt, 1.0)
        # A five-joint leg cannot choose the sole's HEADING as well as its position and
        # tilt, and asking it to is what makes the solve stall short of the target. Drop
        # the yaw row: the sole still lands where it is told, flat, facing wherever the
        # rest of the leg leaves it.
        if 'hip_yaw' not in LEG_ORDER:
            n_sole = _d.site_xmat[s].reshape(3, 3)[:, 2]
            w = w - float(w @ n_sole) * n_sole
        err = np.concatenate([e_p, w])
        if np.linalg.norm(e_p) < 1e-7 and np.linalg.norm(w) < 1e-6:
            break
        mujoco.mj_jacSite(_m, _d, jacp, jacr, s)
        Jfull = np.vstack([jacp, jacr])
        J = Jfull[:, dofs].copy()
        for p, terms in COUPLING.items():
            for a_name, k in terms:
                if a_name in names:
                    J[:, names.index(a_name)] += k * Jfull[:, dadr(p)]
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-8 * np.eye(6), err)
        dq = np.clip(dq, -0.2, 0.2)
        for k, (n, v) in enumerate(zip(names, dq)):
            _d.qpos[qadr(n)] = min(max(_d.qpos[qadr(n)] + v, lim[k, 0]), lim[k, 1])
        _couple()
        mujoco.mj_forward(_m, _d)
    s = sid(f'{side}_foot')
    res = {n: float(np.degrees(_d.qpos[qadr(n)])) for n in names}
    pe = float(np.linalg.norm(p_goal - _d.site_xpos[s]))
    dR = R_goal @ _d.site_xmat[s].reshape(3, 3).T
    mujoco.mju_mat2Quat(qt := np.zeros(4), dR.flatten())
    mujoco.mju_quat2Vel(w := np.zeros(3), qt, 1.0)
    if 'hip_yaw' not in LEG_ORDER:          # heading is not this leg's to control
        n_sole = _d.site_xmat[s].reshape(3, 3)[:, 2]
        w = w - float(w @ n_sole) * n_sole
    return res, pe, float(np.linalg.norm(w))


if __name__ == '__main__':
    print('limits (deg):')
    for n in JOINTS:
        print(f'  {n:22s} {LIMITS[n][0]:8.1f} .. {LIMITS[n][1]:8.1f}')
    print('\nrest foot poses:', {k: np.round(v[0], 4).tolist() for k, v in REST.items()})
    for z in (-0.140, -0.150, -0.155, -0.160):
        r, pe, re_ = solve_leg('right', (0.005, -0.036, z))
        print(f'\nz={z}  poserr={pe:.2e} roterr={re_:.2e}')
        for n, v in r.items():
            lo, hi = LIMITS[n]
            print(f'    {n:22s} {v:8.2f}  {"OK" if lo - 1e-6 <= v <= hi + 1e-6 else "LIMIT!"}')
