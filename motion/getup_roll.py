#!/usr/bin/env python3
"""Find a roll-over, so whichever rise is verified can take over from either start.

microban needed face-up -> face-down: its prone rise was the good one. OP3 is the other
way round -- its hip yaw is +-90, which rules out the straddle the prone rise depends
on, while the supine rise works -- so it needs face-down -> face-up. Pass --to supine.

Rolling is asymmetric and needs momentum, so neither the mirror-symmetric roadmap nor
static stability applies here. This is a plain beam search in the simulator: from the
current state, try many randomly perturbed whole-body poses in parallel, keep the ones
that turn the chest further towards the floor, repeat.

Scored once per candidate, that search finds rolls that only just work. The Mini's first
one turned over perfectly from the nominal state and failed 7 times in 12 under the 0.3
deg joint offsets and 1 deg start tilt sim_check applies -- a momentum move sitting right
on its bifurcation, either completing or not with nothing in between. So --robust R
carries R independently perturbed copies of the robot through the beam together and
scores every candidate on its WORST copy. A roll that survives that is one whose margin
is real rather than an artefact of starting from the exact pose it was searched from.
"""
import argparse
import math
import multiprocessing as mp
import os
import pickle
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

_S = None


def _init():
    global _S
    import tempfile
    import sim_check as SC, build_project as B
    scene = os.path.join(tempfile.gettempdir(), f'roll_{os.getpid()}.xml')
    sim = SC.Sim(SC.build_scene(scene))
    import mujoco
    lo, hi = [], []
    for n in sim.names:
        j = mujoco.mj_name2id(sim.m, mujoco.mjtObj.mjOBJ_JOINT, n)
        lo.append(math.degrees(sim.m.jnt_range[j][0]))
        hi.append(math.degrees(sim.m.jnt_range[j][1]))
    _S = dict(SC=SC, B=B, sim=sim, lo=np.array(lo), hi=np.array(hi), mujoco=mujoco)


def _metrics():
    sim = _S['sim']; d = sim.d
    R = d.xmat[1].reshape(3, 3)
    # chest normal is trunk +x: +1 face-up, -1 face-down
    # tilt = how far the robot is lying on a side rather than squarely on its front;
    # the prone get-up needs both soles touching, which a side-lying finish breaks
    return dict(chest=float(R[2, 0]), up=float(R[2, 2]),
                tilt=abs(float(R[2, 1])),
                comz=float(d.subtree_com[0][2]) * 1000,
                roll=float(math.degrees(math.atan2(R[2, 1], R[2, 2]))))


def _settle_start(task):
    """Drop the robot on the face it is starting from, arms flat at its sides.

    Replica 0 is nominal; the rest get the joint offsets and start tilt sim_check uses,
    drawn from the same seed sequence so a run is reproducible.
    """
    to, rep = task
    S = _S; sim = S['sim']; B = S['B']; mujoco = S['mujoco']
    q = {n: 0.0 for n in B.JOINT_ORDER}
    # Arms out to the sides on the back, tucked in on the front -- the same two resting
    # poses the get-ups start from, so the roll hands over to them cleanly.
    q.update(B.armset(0, 150 if to == 'prone' else 90, 0))
    if rep == 0:
        off, tilt = np.zeros(len(sim.names)), (0., 0.)
    else:
        rng = np.random.default_rng(7717 + rep)
        off = rng.normal(0, S['SC'].OFFSET_SD, len(sim.names))
        tilt = tuple(rng.normal(0, S['SC'].TILT_SD, 2))
    cmd = sim._settle(q, tilt, -90.0 if to == 'prone' else 90.0)
    return dict(qpos=sim.d.qpos.copy().tolist(), qvel=sim.d.qvel.copy().tolist(),
                cmd=cmd.tolist(), off=off.tolist(), mt=_metrics())


def _try(st, target, frames, hold):
    S = _S; sim = S['sim']; mujoco = S['mujoco']; m, d = sim.m, sim.d
    d.qpos[:] = st['qpos']; d.qvel[:] = st['qvel']
    mujoco.mj_forward(m, d)
    a = np.array(st['cmd']); b = np.array(target)
    steps = max(1, round(1.0 / 100.0 / m.opt.timestep))
    lim = math.degrees(S['SC'].MAX_SPEED) / 100.0
    off = np.array(st['off'])
    last = a.copy()
    for f in range(1, frames + 1):
        u = f / frames
        c = a + (b - a) * (-(math.cos(math.pi * u) - 1) / 2)
        c = np.clip(c, last - lim, last + lim); last = c.copy()
        d.ctrl[:] = np.deg2rad(c + off)
        for _ in range(steps):
            mujoco.mj_step(m, d)
    for _ in range(int(hold / m.opt.timestep)):
        mujoco.mj_step(m, d)
    return dict(qpos=d.qpos.copy().tolist(), qvel=d.qvel.copy().tolist(),
                cmd=last.tolist(), off=st['off'], mt=_metrics())


def score(mt, want):
    """Lower is better: chest turned all the way over AND the body square."""
    return -want * mt['chest'] + 0.8 * mt['tilt']


def _fan(task):
    """Draw candidate poses around the current command and simulate each replica."""
    states, n, spread, frames, hold, seed, want = task
    S = _S
    rng = np.random.default_rng(seed)
    lo, hi = S['lo'], S['hi']
    out = []
    base = np.array(states[0]['cmd'])
    for k in range(n):
        d = rng.normal(0, spread, len(base))
        # hips and shoulders get the big excursions -- they are what rolls the trunk
        for i, nm in enumerate(S['sim'].names):
            if 'hipjoint_zy' in nm or 'hipjoint_xr' in nm or 'shoulder' in nm:
                d[i] *= 2.2
        tgt = np.clip(base + d, lo, hi)
        # Every replica is driven to the SAME target -- one motion, R robots. The
        # candidate is worth what its worst robot gets, not what its luckiest one does.
        res = [_try(st, tgt.tolist(), frames, hold) for st in states]
        out.append((tgt.tolist(), res, max(score(r['mt'], want) for r in res)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--beam', type=int, default=8)
    ap.add_argument('--fan', type=int, default=240)
    ap.add_argument('--steps', type=int, default=14)
    ap.add_argument('--frames', type=int, default=60)
    ap.add_argument('--hold', type=float, default=1.0,
                    help='seconds to let the robot settle before a candidate is scored. '
                         'Not padding: a roll can be over at 0.3 s and back on its front '
                         'at 1.0 s. The Mini found one that scored +0.99 chest and was at '
                         '-0.21 a second later. Score what the robot is still doing when '
                         'the next keyframe starts, not what it passes through.')
    ap.add_argument('--spread', type=float, default=28.0, help='deg, candidate spread')
    ap.add_argument('--workers', type=int, default=max(1, mp.cpu_count() - 2))
    ap.add_argument('--robust', type=int, default=1,
                    help='how many independently perturbed copies of the robot to carry '
                         'through the search. Each candidate is scored on its WORST copy, '
                         'so what survives is a roll with margin rather than one that '
                         'happens to work from the exact state it was searched from. '
                         'Costs a factor of R in simulation.')
    ap.add_argument('--to', choices=['prone', 'supine'], default='prone',
                    help='which way to roll: onto the front (microban) or onto the '
                         'back (OP3, whose supine rise is the verified one)')
    ap.add_argument('--out', default='motion/log/roll_over.pkl')
    args = ap.parse_args()

    # +1 when the goal is chest-up, -1 when chest-down; every objective below is written
    # for 'drive chest towards the goal' and multiplied by this.
    WANT = 1.0 if args.to == 'supine' else -1.0

    pool = mp.Pool(args.workers, initializer=_init)
    states = pool.map(_settle_start, [(args.to, r) for r in range(max(1, args.robust))])
    mt = states[0]['mt']
    print(f'start ({"prone" if args.to == "supine" else "supine"}): '
          f'chest {mt["chest"]:+.2f} up {mt["up"]:+.2f} comZ {mt["comz"]:.0f} '
          f'| {len(states)} perturbed cop{"y" if len(states) == 1 else "ies"}', flush=True)
    beam = [dict(states=states, mt=mt, worst=score(mt, WANT), seq=[])]
    per = max(1, args.fan // args.workers)
    for step in range(args.steps):
        tasks, owner = [], []
        for bi, b in enumerate(beam):
            for w in range(args.workers):
                tasks.append((b['states'], per, args.spread, args.frames, args.hold,
                              step * 1009 + bi * 31 + w, WANT))
                owner.append(bi)
        out = pool.map(_fan, tasks)
        cands = []
        for bi, part in zip(owner, out):
            for tgt, res, worst in part:
                cands.append(dict(states=res, mt=res[0]['mt'], worst=worst,
                                  seq=beam[bi]['seq'] + [tgt]))
        # want the chest turned all the way over AND the body square, not propped on
        # one side -- a side-lying finish leaves the rise without both soles down
        cands.sort(key=lambda z: z['worst'])
        uniq, keys = [], []
        for c in cands:
            k = round(c['mt']['chest'], 2), round(c['mt']['tilt'], 2)
            if k in keys:
                continue
            keys.append(k); uniq.append(c)
        beam = uniq[:args.beam]
        b0, w0 = beam[0]['mt'], beam[0]['worst']
        wm = min((r['mt'] for r in beam[0]['states']),
                 key=lambda m: WANT * m['chest'] - 0.8 * m['tilt'])
        print(f'  step {step}: {len(cands)} tried | nominal chest {b0["chest"]:+.2f} '
              f'tilt {b0["tilt"]:.2f} | worst copy chest {wm["chest"]:+.2f} '
              f'tilt {wm["tilt"]:.2f}', flush=True)
        # The stop test reads the WORST copy: every copy has to be over and square.
        if -w0 > 0.90 - 0.8 * 0.12 and all(WANT * r['mt']['chest'] > 0.90
                                           and r['mt']['tilt'] < 0.12
                                           for r in beam[0]['states']):
            print(f'  every copy square, face {"up" if WANT > 0 else "down"}'); break
    pool.close()
    best = min(beam, key=lambda z: z['worst'])
    wm = min((r['mt'] for r in best['states']),
             key=lambda m: WANT * m['chest'] - 0.8 * m['tilt'])
    print(f'final: nominal chest {best["mt"]["chest"]:+.2f} tilt {best["mt"]["tilt"]:.2f} '
          f'| worst copy chest {wm["chest"]:+.2f} tilt {wm["tilt"]:.2f} '
          f'after {len(best["seq"])} poses')
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    pickle.dump(best['seq'], open(args.out, 'wb'))
    print('saved ->', args.out)


if __name__ == '__main__':
    main()
