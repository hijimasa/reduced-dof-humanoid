#!/usr/bin/env python3
"""Search a get-up that starts face-up and never rolls over.

The existing supine action rolls onto its front and reuses the prone rise, which costs
2.6 s and looks bad. Getting up directly is harder than it sounds: pressing on the arms
alone pitches the trunk to about 45 deg and stops, because the feet are still ~110 mm
in front of the centre of mass and nothing pulls them back.

So the score here is not "how upright" but "how upright AND how close the CoM is to the
feet" -- the sit-up has to be paid for by folding the legs in, not by pushing harder.
Candidates are sampled freely in joint space (mirror-symmetric) around the pose the
robot is currently commanded to hold, rather than drawn from the flat-foot roadmap:
none of the sitting or tucked poses this needs are in that roadmap, they are only
statically stable on the hands and the backside.
"""
import argparse, math, os, pickle, sys, tempfile
import multiprocessing as mp
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from joint_names import jn, LEG_ORDER, LEG_MIRROR, NLEG, NPARAM

NAMES = list(LEG_ORDER)
MIRROR = np.array(LEG_MIRROR)
_S = None


def _init(stress=None):
    global _S
    import sim_check as SC, build_project as B
    if stress:
        # Rank candidates against the disturbance the robot will really meet, not the
        # nominal one. The prone rise this replaces gets up 40/40 at 0.3 deg / 1 deg and
        # 26/40 at 0.8 deg / 2.5 deg, and no retiming fixes that -- slowing it down 2.4x
        # only reaches 29/40. The fragility is in the trajectory, so it has to be
        # searched out.
        SC.OFFSET_SD, SC.TILT_SD = stress
    scene = os.path.join(tempfile.gettempdir(), f'sup_{os.getpid()}.xml')
    sim = SC.Sim(SC.build_scene(scene))
    _S = dict(SC=SC, B=B, sim=sim)


def pose_dict(x):
    """Search vector (leg, mirrored + shoulder pitch/roll + elbow) -> joint angles, deg.

    Delegates to build_project.getup_pose so the search explores exactly the poses the
    generated project can express: same mirroring, same per-joint sign convention, same
    shoulder zero. Searching a different space and then converting is how you get a
    get-up that worked in the search and not in the project.
    """
    return _S['B'].getup_pose(list(x[:NLEG]), tuple(x[NLEG:NPARAM]))


def metrics():
    sim = _S['sim']; d = sim.d
    R = d.xmat[1].reshape(3, 3)
    pitch = math.degrees(math.atan2(-R[2, 0], R[2, 2]))
    fp = [d.site_xpos[sim.sfoot[s]] for s in ('right', 'left')]
    mid = (fp[0] + fp[1]) / 2
    com = d.subtree_com[0]
    sole = float(np.mean([d.site_xmat[sim.sfoot[s]].reshape(3, 3)[2, 2]
                          for s in ('right', 'left')]))
    Rf = d.site_xmat[sim.sfoot['right']].reshape(3, 3)
    fy = math.degrees(math.atan2(Rf[1, 0], Rf[0, 0]) - math.atan2(R[1, 0], R[0, 0]))
    return dict(up=float(R[2, 2]), pitch=pitch, comz=float(com[2]) * 1000,
                sole=sole, footyaw=(fy + 180) % 360 - 180,
                dcom=float(np.linalg.norm((com - mid)[:2])) * 1000,
                footz=float(min(p[2] for p in fp)) * 1000)


def _bounds():
    """Bounds on the search parameters, inverted through pose_dict's own mapping.

    A parameter is not a joint: one entry drives two joints, mirrored, each with its own
    sign, and the shoulder roll additionally sits on a non-zero zero (OP3 models its
    shoulders in a T-pose). Rather than restate that algebra here and have it drift, the
    mapping is measured -- pose_dict is linear in every parameter, so its offset and
    slope per joint come straight from evaluating it at 0 and at 1.
    """
    from ik import LIMITS
    base = pose_dict(np.zeros(NPARAM))
    lo, hi = [], []
    for i in range(NPARAM):
        e = np.zeros(NPARAM)
        e[i] = 1.0
        col = pose_dict(e)
        a, b = -1e9, 1e9
        for n, v in col.items():
            slope = v - base[n]
            if abs(slope) < 1e-9:
                continue
            jl, jh = LIMITS[n]
            p, q = sorted(((jl - base[n]) / slope, (jh - base[n]) / slope))
            a, b = max(a, p), min(b, q)
        lo.append(a); hi.append(b)
    return np.array(lo), np.array(hi)


def _settle(pitch0):
    """Drop the robot down with the arms flat at its sides and let it come to rest."""
    B = _S['B']; sim = _S['sim']
    x = np.zeros(NPARAM)
    x[NLEG + 1] = 150.0 if pitch0 < 0 else 90.0   # the shoulder roll   # arms out flat; narrower when face-down

    cmd = sim._settle(pose_dict(x), (0.0, 0.0), pitch0)
    return sim.d.qpos.copy().tolist(), sim.d.qvel.copy().tolist(), cmd.tolist(), x


def _run(task):
    import mujoco
    qpos, qvel, cmd, target, frames, hold, seed = task
    sim = _S['sim']; m, d = sim.m, sim.d
    d.qpos[:] = qpos; d.qvel[:] = qvel
    mujoco.mj_forward(m, d)
    off = (np.zeros(len(sim.names)) if seed is None
           else np.random.default_rng(seed).normal(0, _S['SC'].OFFSET_SD, len(sim.names)))
    a = np.array(cmd)
    b = np.array([pose_dict(np.asarray(target))[n] for n in sim.names])
    steps = max(1, round(1.0 / 100.0 / m.opt.timestep))
    lim = math.degrees(_S['SC'].MAX_SPEED) / 100.0
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
    return (d.qpos.copy().tolist(), d.qvel.copy().tolist(), last.tolist(), metrics())


def _probe(pitch0=-90.0):
    _init()
    _settle(pitch0)
    return metrics()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--beam', type=int, default=8)
    ap.add_argument('--fan', type=int, default=90)
    ap.add_argument('--steps', type=int, default=16)
    ap.add_argument('--frames', type=int, default=45)
    ap.add_argument('--hold', type=float, default=0.2)
    ap.add_argument('--radius', type=float, default=55.0)
    ap.add_argument('--robust', type=int, default=2)
    ap.add_argument('--workers', type=int, default=max(1, mp.cpu_count() - 2))
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--nodes', default='motion/log/getup_nodes.pkl')
    ap.add_argument('--stress', default='', help='"offset_sd,tilt_sd" in degrees')
    ap.add_argument('--pitch', type=float, default=-90.0,
                    help='trunk pitch at t=0: -90 face-up, +90 face-down')
    ap.add_argument('--out', default='motion/log/getup_supine.pkl')
    args = ap.parse_args()

    ROAD = None
    road_file = args.nodes

    stress = tuple(float(v) for v in args.stress.split(',')) if args.stress else None
    pool = mp.Pool(args.workers, initializer=_init, initargs=(stress,))
    _init(stress)
    LO, HI = _bounds()
    if os.path.exists(road_file):
        nd = pickle.load(open(road_file, 'rb'))
        ROAD = np.array([np.degrees(np.concatenate([x[0][:NLEG], x[0][NLEG:NPARAM]])) for x in nd])
        # The roadmap on disk was built for whichever robot ran getup_search.py last, and
        # nothing in the file says which. Poses it holds that this robot cannot reach are
        # not candidates: unfiltered, microban's roadmap hands OP3 a -240 deg hip yaw and
        # the search returns a get-up the project then refuses to emit.
        keep = np.all((ROAD >= LO - 1e-9) & (ROAD <= HI + 1e-9), axis=1)
        if not keep.all():
            print(f'roadmap: dropped {int((~keep).sum())} of {len(ROAD)} poses '
                  f'outside this robot\'s joint limits')
        ROAD = ROAD[keep]
        print(f'{len(ROAD)} roadmap poses available for the final rise')
    qpos, qvel, cmd, x0 = _settle(args.pitch)
    m0 = metrics()
    print(f'settled at pitch {args.pitch:+.0f}:',
          {k: round(v, 1) for k, v in m0.items()}, flush=True)

    def score(mt, phase=None):
        """One monotone objective, not a sequence of phases.

        Staged scores kept flipping the beam into whatever the newest stage rewarded --
        chase "CoM over the feet" on its own and the robot happily rolls back down with
        its knees tucked, because that scores wonderfully. Every term is on all the time
        instead, so height only counts when it is paid for with a flat sole under the
        mass: a kneel on the backs of the feet (comZ 86, sole -0.9, CoM 100 mm out)
        scores 11, sitting scores 23, and standing scores 183.
        """
        return (mt['comz'] - 0.6 * abs(mt['pitch']) - 0.35 * mt['dcom']
                + 45.0 * mt['sole'] + 30.0 * mt['up'] - 0.15 * abs(mt['footyaw']))

    beam = [dict(qpos=qpos, qvel=qvel, cmd=cmd, x=x0.copy(), mt=m0, seq=[])]
    phase = 1
    rng = np.random.default_rng(args.seed)
    for step in range(args.steps):
        tasks, owner = [], []
        for bi, b in enumerate(beam):
            scale = rng.choice([0.35, 0.7, 1.0], args.fan)[:, None]
            cand = np.clip(b['x'] + rng.normal(0, args.radius, (args.fan, NPARAM)) * scale,
                           LO, HI)
            if ROAD is not None and b['mt']['up'] > 0.6:
                sel = ROAD[np.abs(ROAD - b['x']).max(axis=1) < 90.0]
                if len(sel):
                    k = min(len(sel), args.fan // 2)
                    cand = np.vstack([cand[:args.fan - k], sel[rng.choice(len(sel), k, False)]])
            for c in cand:
                tasks.append((b['qpos'], b['qvel'], b['cmd'], c.tolist(),
                              args.frames, args.hold, None))
                owner.append((bi, c))
        out = pool.map(_run, tasks)
        cands = []
        for (bi, c), (qp, qv, cm, mt) in zip(owner, out):
            prev = beam[bi]['mt']
            if mt['sole'] < -0.35:          # standing on the backs of the feet
                continue
            if score(mt) < score(prev) - 25:
                continue
            cands.append(dict(qpos=qp, qvel=qv, cmd=cm, x=c, mt=mt,
                              seq=beam[bi]['seq'] + [c.tolist()]))
        if not cands:
            print(f'  step {step}: dead end'); break
        cands.sort(key=lambda z: -score(z['mt']))
        head = cands[:max(args.beam * 3, 16)]
        for c in head + cands[len(head):]:
            c['rank'] = score(c['mt'])
        if args.robust:
            # Re-run each front runner with servo-offset draws and rank it by the WORST
            # of (nominal, mean perturbed). Scoring the nominal run alone picks
            # transitions that need perfect servos: the first version of this motion
            # reached the beam's goal on the nominal run and then got up in 10 of 16
            # trials, and no amount of retiming fixed that -- the trajectory itself was
            # the marginal part, so robustness has to be inside the search.
            rt = [(c['qpos'], c['qvel'], c['cmd'], c['x'].tolist(), args.frames,
                   args.hold, k + 1 + step * 97) for c in head for k in range(args.robust)]
            ro = pool.map(_run, rt)
            for i, c in enumerate(head):
                got = [g[3] for g in ro[i * args.robust:(i + 1) * args.robust]]
                c['rank'] = min(score(c['mt']), float(np.mean([score(g) for g in got])))
            head.sort(key=lambda z: -z['rank'])
        uniq, keys = [], set()
        for c in head + sorted(cands[len(head):], key=lambda z: -z['rank']):
            k = (round(c['mt']['pitch'] / 6), round(c['mt']['comz'] / 6),
                 round(c['mt']['dcom'] / 8))
            if k in keys:
                continue
            keys.add(k); uniq.append(c)
        beam = uniq[:args.beam]
        b0 = beam[0]['mt']
        print(f'  step {step}: {len(cands)} viable | pitch {b0["pitch"]:6.1f} '
              f'comZ {b0["comz"]:6.1f} dcom {b0["dcom"]:6.1f} sole {b0["sole"]:5.2f} '
              f'up {b0["up"]:5.2f} score {score(b0):6.1f}', flush=True)
        if b0['up'] > 0.92 and b0['comz'] > 138 and b0['sole'] > 0.85 and b0['dcom'] < 40:
            print('  standing'); break
    pool.close()
    best = max(beam, key=lambda z: z.get('rank', score(z['mt'])))
    print('final:', {k: round(v, 1) for k, v in best['mt'].items()})
    pickle.dump(best['seq'], open(args.out, 'wb'))
    print(f'saved {len(best["seq"])} poses -> {args.out}')


if __name__ == '__main__':
    main()
