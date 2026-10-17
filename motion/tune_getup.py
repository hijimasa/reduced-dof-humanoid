#!/usr/bin/env python3
"""Shrink a get-up to the fastest frame counts that still work every time.

The rise keyframes were authored at 280 frames (2.8 s) each because the beam search
that found them ran at a fixed cadence, not because the transitions need that long.
This searches the per-keyframe durations directly: minimise total time subject to
surviving every perturbation trial from the correct starting pose.

The lower bound per keyframe is not free -- LME's speed limiter clips a command that
moves a joint faster than the servo can follow, so a keyframe whose largest joint
travel is D degrees cannot be shorter than D / (max_speed / fps) frames.
"""
import argparse, math, os, sys, tempfile, time
import multiprocessing as mp
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

_S = None


def _init(action, scale_jitter=0.0):
    global _S
    import sim_check as SC, build_project as B
    SC.SCALE_JITTER = scale_jitter
    scene = os.path.join(tempfile.gettempdir(), f'getup_{os.getpid()}.xml')
    _S = dict(SC=SC, B=B, sim=SC.Sim(SC.build_scene(scene)), seq=_base(B, action))


def _base(B, action):
    keys = getattr(B, 'STANDUP_' + action.upper()) + [('home', B.HOME, 60)]
    return [(n, q, f) for n, q, f in keys]


def _eval(task):
    frames, seeds, pitch0 = task
    B, SC, sim = _S['B'], _S['SC'], _S['sim']
    ez = {n: '3: easeInOutSine' for n in B.JOINT_ORDER}
    seq = [(n, int(f), q, ez) for (n, q, _), f in zip(_S['seq'], frames)]
    if 'ref' not in _S:
        sim._settle(B.HOME, (0.0, 0.0))
        _S['ref'] = float(sim.d.subtree_com[0][2]) * 1000
    npass = 0
    for k, s in enumerate(seeds):
        rng = np.random.default_rng(s)
        off = (np.zeros(len(sim.names)) if s == 0
               else rng.normal(0, SC.OFFSET_SD, len(sim.names)))
        tilt = (0.0, 0.0) if s == 0 else tuple(rng.normal(0, SC.TILT_SD, 2))
        # Spread the perturbed seeds across the playback-speed band, as sim_check does:
        # Studio plays a motion at whatever rate it manages, and a get-up that only
        # works at exactly 1.0x is not one that works.
        j = SC.SCALE_JITTER
        sim.scale = 1.0 if (not j or k == 0 or len(seeds) < 3) else 1.0 - j + 2 * j * (k - 1) / (len(seeds) - 2)
        r = sim.play(seq, 100.0, off, tilt, pitch0=pitch0)
        # not falling is not the same as standing up: a robot that ends on its knees
        # has an upright trunk too, so check the centre of mass reached standing height
        comz = float(sim.d.subtree_com[0][2]) * 1000
        npass += int(r['ok'] and comz > 0.80 * _S['ref'])
    return dict(ok=npass / len(seeds), t=sum(int(f) for f in frames) / 100.0)


# A get-up either happens or it does not, and unlike the walk there is no reason to
# accept less than every trial: the robot is on the floor with all the time in the world.
RATE_FLOOR = 1.0


def fitness(r):
    return -r['t'] - 500.0 * (RATE_FLOOR - min(r['ok'], RATE_FLOOR))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('action', choices=['face_down', 'face_up'])
    ap.add_argument('--pop', type=int, default=90)
    ap.add_argument('--elite', type=int, default=12)
    ap.add_argument('--gens', type=int, default=25)
    ap.add_argument('--trials', type=int, default=6)
    ap.add_argument('--verify', type=int, default=40)
    ap.add_argument('--hi-scale', type=float, default=1.3,
                    help='upper bound per keyframe, as a multiple of its current frames. '
                         'Raise it when the starting schedule is too quick to be robust '
                         'and the search needs room to slow down, not just speed up.')
    ap.add_argument('--workers', type=int, default=max(1, mp.cpu_count() - 2))
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--scale-jitter', type=float, default=0.0,
                    help='also require the schedule to survive playback speeds in U(1-j, 1+j)')
    args = ap.parse_args()

    import sim_check as SC, build_project as B
    SC.SCALE_JITTER = args.scale_jitter
    seq = _base(B, args.action)
    pitch0 = SC.START_PITCH['standup_' + args.action]
    step_lim = math.degrees(SC.MAX_SPEED) / 100.0

    # per-keyframe floor from the servo speed limit, with 15% headroom
    lo = []
    prev = seq[0][1]
    for n, q, f in seq:
        d = max(abs(q[j] - prev[j]) for j in B.JOINT_ORDER)
        lo.append(max(8.0, 1.15 * d / step_lim))
        prev = q
    lo = np.array(lo)
    hi = np.maximum(lo + 10, np.array([f for _, _, f in seq], float) * args.hi_scale)
    print(f'{len(seq)} keyframes, current {sum(f for _,_,f in seq)/100:.2f} s, '
          f'speed-limit floor {lo.sum()/100:.2f} s')

    rng = np.random.default_rng(args.seed)
    mu = np.array([f for _, _, f in seq], float)
    sd = np.maximum((hi - lo) / 3.0, 5.0)

    pool = mp.Pool(args.workers, initializer=_init, initargs=(args.action, args.scale_jitter))
    best = (mu.copy(), -1e9, None)
    archive = []
    t0 = time.time()
    for g in range(args.gens):
        X = np.round(np.clip(rng.normal(mu, sd, (args.pop, len(seq))), lo, hi))
        if g == 0:
            X[0] = np.round(np.clip(mu, lo, hi))
        else:
            X[0] = best[0]
        # Successive halving, for the same reason as in tune_gait: a schedule that
        # clears 8 perturbation draws routinely fails 40, so screen cheaply and pay
        # for a real estimate only on the survivors.
        idx = np.arange(args.pop)
        res = None
        for ntr, keep in ((args.trials, args.pop // 3),
                          (args.trials * 3, args.elite * 2),
                          (args.trials * 7, args.elite)):
            seeds = [0] + list(rng.integers(1, 10 ** 6, ntr - 1))
            res = pool.map(_eval, [(X[i], seeds, pitch0) for i in idx])
            f = np.array([fitness(r) for r in res])
            o = np.argsort(-f)
            if keep >= len(idx):
                idx, res = idx[o], [res[i] for i in o]
                break
            idx, res = idx[o[:keep]], [res[i] for i in o[:keep]]
        fit = np.array([fitness(r) for r in res])
        if fit[0] > best[1]:
            best = (X[idx[0]], float(fit[0]), res[0])
        for j, i in enumerate(idx):
            if res[j]['ok'] >= RATE_FLOOR:
                archive.append((X[i], res[j]['t']))
        el = X[idx[:args.elite]]
        mu = el.mean(axis=0)
        sd = np.maximum(el.std(axis=0), (hi - lo) * 0.05)
        b = res[0]
        print(f'gen {g:2d}  best {b["t"]:5.2f}s pass {b["ok"]*100:5.0f}%  '
              f'incumbent {best[2]["t"]:5.2f}s  [{time.time()-t0:4.0f}s]', flush=True)
    # re-run the shortest survivors over many fresh trials: a get-up that clears six
    # perturbation draws can still be a coin flip
    archive.sort(key=lambda z: z[1])
    fin, seen = [], set()
    for x, t in archive:
        k = tuple(x)
        if k in seen:
            continue
        seen.add(k); fin.append(x)
        if len(fin) >= 24:
            break
    if fin:
        vs = [0] + list(rng.integers(1, 10 ** 6, args.verify - 1))
        vr = pool.map(_eval, [(x, vs, pitch0) for x in fin])
        cl = [(x, r) for x, r in zip(fin, vr) if r['ok'] >= RATE_FLOOR]
        print(f'\nverify: {len(cl)}/{len(fin)} finalists hold over {args.verify} trials')
        if cl:
            x, r = min(cl, key=lambda z: z[1]['t'])
            best = (x, fitness(r), r)
    pool.close()
    print(f'\nbest {best[2]["t"]:.2f} s (was {sum(f for _,_,f in seq)/100:.2f} s), '
          f'pass {best[2]["ok"]*100:.0f}%')
    print('frames = [' + ', '.join(str(int(v)) for v in best[0]) + ']')
    for (n, _, f), v in zip(seq, best[0]):
        print(f'  {n:14s} {f:4d} -> {int(v):4d}')


if __name__ == '__main__':
    main()
