#!/usr/bin/env python3
"""Search gait parameters for the fastest motion that still never falls.

The hand-tuned numbers in build_project.py were picked to be safe, not fast: the
lateral weight transfer alone takes 0.85 s, so a walk cycle is 3.1 s for 55 mm.
This runs a cross-entropy search over the gait parameters with the same physics
sim_check uses, scoring speed but rejecting outright anything that falls in any
perturbation trial.

Both halves of the constraint matter. Scoring speed alone finds gaits that stride
100 mm and topple on the third cycle; scoring survival alone keeps the status quo.
"""
import argparse, math, os, sys, tempfile, time
import multiprocessing as mp
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ---- parameter spaces: name -> (lo, hi, integer?) ------------------------------
from robot_profile import profile

_P = profile()
GLOBAL, WALK = _P.GAIT_GLOBAL, _P.GAIT_WALK
SIDE, TURN = _P.GAIT_SIDE, _P.GAIT_TURN

SPACES = {'walk': {**GLOBAL, **WALK}, 'side': SIDE, 'turn': TURN}

_S = None


def _init(fixed, stress=None, jitter=None):
    global _S
    import sim_check as SC, build_project as B
    if stress:
        # Tune against a disturbance the robot will actually meet. With the nominal
        # 0.3 deg / 1 deg draws the search spends every bit of margin above that level
        # on speed, and the result is quick in the harness and unreliable in Studio --
        # measured, the fast gait held 96% at the nominal level and only 46% at 0.8/2.5,
        # worse there than the slow gait it replaced.
        SC.OFFSET_SD, SC.TILT_SD = stress
    if jitter is not None:
        SC.SCALE_JITTER = jitter
    scene = os.path.join(tempfile.gettempdir(), f'tune_{os.getpid()}.xml')
    _S = dict(SC=SC, B=B, sim=SC.Sim(SC.build_scene(scene)), fixed=fixed)


def _apply(p):
    """Push a parameter dict into build_project's module globals."""
    B = _S['B']
    for k, v in {**_S['fixed'], **p}.items():
        if k in ('ARM', 'STEP', 'SIDE', 'PHI'):
            continue
        setattr(B, k, v)
    B._cache.clear()
    B.HOME = B.pose((B.X0, -B.Y0, B.Z0), (B.X0, B.Y0, B.Z0))
    B.CROUCH = B.pose((B.X0, -B.Y0, B.Z0 + 0.012), (B.X0, B.Y0, B.Z0 + 0.012))


def _seq(keys, loops):
    """cyclic_action's graph, flattened the way sim_check.expand would."""
    B = _S['B']
    ez = {n: '3: easeInOutSine' for n in B.JOINT_ORDER}
    out = [('home', B.F_HOME, B.HOME, ez)]
    for _ in range(loops + 1):
        out += [(n, f, q, ez) for n, q, f in keys]
    out.append(('exit_home', B.F_HOME, B.HOME, ez))
    cyc = sum(f for _, q, f in keys) * (loops + 1) / 100.0
    return out, cyc


def _eval(task):
    """One candidate: play every variant of the gait over several trials."""
    family, p, seeds, loops = task
    B, SC, sim = _S['B'], _S['SC'], _S['sim']
    try:
        _apply(p)
        if family == 'walk':
            variants = [B.walk_cycle(p['STEP'], p['ARM']), B.walk_cycle(-p['STEP'], p['ARM'])]
        elif family == 'side':
            variants = [B.sidestep_cycle(p['SIDE']), B.sidestep_cycle(-p['SIDE'])]
        elif B.TURN_SLIP:
            # A leg with no hip yaw turns by stepping asymmetrically, so the parameters
            # are a mean stride and the difference between the two legs, not an angle.
            mean, d = p['STEP_T'], p['DSTEP']
            variants = [B.turn_slip_cycle(mean + d / 2, mean - d / 2),
                        B.turn_slip_cycle(mean - d / 2, mean + d / 2)]
        else:
            variants = [B.turn_cycle(p['PHI']), B.turn_cycle(-p['PHI'])]
        for keys in variants:
            for n, q, f in keys:
                B.check(family, q)
    except BaseException:
        return dict(ok=0.0, speed=0.0, margin=0.0, why='infeasible')
    npass = tot = 0
    speeds, margin = [], 1.0
    for keys in variants:
        seq, cyc = _seq(keys, loops)
        for s in seeds:
            rng = np.random.default_rng(s)
            off = (np.zeros(len(sim.names)) if s == 0
                   else rng.normal(0, SC.OFFSET_SD, len(sim.names)))
            tilt = (0.0, 0.0) if s == 0 else tuple(rng.normal(0, SC.TILT_SD, 2))
            # Spread the trials across the playback-speed band the motion has to survive,
            # deterministically so that a 5-trial candidate is judged on the whole band
            # and not on wherever the draws happened to land. Studio's real frame rate is
            # not the nominal one, so a gait scored only at 1.0 is scored at a speed it
            # will rarely be played at.
            if SC.SCALE_JITTER and s != 0 and len(seeds) > 1:
                f = (list(seeds).index(s) - 1) / max(1, len(seeds) - 2)
                sim.scale = 1.0 - SC.SCALE_JITTER + 2 * SC.SCALE_JITTER * f
            else:
                sim.scale = 1.0
            r = sim.play(seq, 100.0, off, tilt)
            tot += 1
            npass += int(r['ok'])
            margin = min(margin, r['min_up'])
            if r['ok']:
                # net progress, not gross: a walk that covers 400 mm forward while
                # sliding 100 mm sideways is not a faster walk, it is a curved one
                if family == 'turn':
                    v = abs(r['dyaw']) - 12.0 * math.hypot(r['dx'], r['dy'])
                elif family == 'side':
                    v = abs(r['dy']) * 1000 - abs(r['dx']) * 1000 - 0.6 * abs(r['dyaw'])
                else:
                    v = abs(r['dx']) * 1000 - abs(r['dy']) * 1000 - 0.6 * abs(r['dyaw'])
                speeds.append(v / cyc)
    rate = npass / max(1, tot)
    return dict(ok=rate, speed=float(np.mean(speeds)) if speeds else 0.0, margin=margin)


# Survival bar. Measured over 40 trials the ORIGINAL hand-tuned walk survives 96.2%,
# not 100% -- the 8/8 scores it used to get were just a small sample. So demanding a
# perfect score here would reject every gait including the one already shipped. The
# search instead has to be at least as safe as what it replaces.
RATE_FLOOR = 0.96      # overridden by --rate-floor


def fitness(r):
    # below the floor nothing is fast enough to make up for it; above it, faster wins,
    # with the tipping margin as a tiebreak
    return (r['speed'] + 3.0 * r['margin']
            - PENALTY * max(0.0, RATE_FLOOR - r['ok'])
            - 40.0 * (1.0 - min(r['ok'], 1.0)))


PENALTY = 900.0        # fitness cost per unit of shortfall below RATE_FLOOR


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('family', choices=list(SPACES))
    ap.add_argument('--pop', type=int, default=60)
    ap.add_argument('--elite', type=int, default=10)
    ap.add_argument('--gens', type=int, default=12)
    ap.add_argument('--trials', type=int, default=6)
    ap.add_argument('--verify', type=int, default=24,
                    help='trials used to re-score the finalists (guards against a '
                         'config that only suits the seeds it was searched with)')
    ap.add_argument('--loops', type=int, default=2)
    ap.add_argument('--workers', type=int, default=max(1, mp.cpu_count() - 2))
    ap.add_argument('--fixed', default='', help='k=v,... held constant (e.g. tuned globals)')
    ap.add_argument('--init', default='', help='k=v,... seed for the initial mean')
    ap.add_argument('--scale-jitter', type=float, default=0.0,
                    help='tune against playback speeds drawn across U(1-j, 1+j) instead '
                         'of only the nominal rate')
    ap.add_argument('--stress', default='',
                    help='"offset_sd,tilt_sd" in degrees, replacing sim_check\'s nominal '
                         '0.3,1.0 for the whole search')
    ap.add_argument('--rate-floor', type=float, default=None)
    ap.add_argument('--penalty', type=float, default=None)
    ap.add_argument('--seed', type=int, default=1)
    args = ap.parse_args()

    global RATE_FLOOR, PENALTY
    if args.rate_floor is not None:
        RATE_FLOOR = args.rate_floor
    if args.penalty is not None:
        PENALTY = args.penalty
    stress = tuple(float(v) for v in args.stress.split(',')) if args.stress else None
    if stress:
        import sim_check as SC
        SC.OFFSET_SD, SC.TILT_SD = stress
        print(f'stress model: servo offset sd {stress[0]} deg, initial tilt sd {stress[1]} deg')
    fixed = {}
    for kv in filter(None, args.fixed.split(',')):
        k, v = kv.split('=')
        fixed[k] = float(v)
    space = {k: v for k, v in SPACES[args.family].items() if k not in fixed}
    keys = list(space)
    lo = np.array([space[k][0] for k in keys], float)
    hi = np.array([space[k][1] for k in keys], float)
    isint = np.array([space[k][2] for k in keys], bool)

    rng = np.random.default_rng(args.seed)
    mu = (lo + hi) / 2
    sd = (hi - lo) / 3.5
    init = {}
    for kv in filter(None, args.init.split(',')):
        k, v = kv.split('=')
        init[k] = float(v)
    for i, k in enumerate(keys):
        if k in init:
            mu[i] = init[k]

    def decode(x):
        x = np.clip(x, lo, hi)
        p = {}
        for i, k in enumerate(keys):
            p[k] = int(round(x[i])) if isint[i] else float(x[i])
        p.update(fixed)
        return p

    pool = mp.Pool(args.workers, initializer=_init,
                   initargs=(fixed, stress, args.scale_jitter))
    best = (None, -1e9, None)
    archive = []
    t0 = time.time()
    for g in range(args.gens):
        X = rng.normal(mu, sd, (args.pop, len(keys)))
        if g == 0 and init:
            X[0] = mu                          # score the hand-tuned config as-is
        if best[0] is not None:
            X[0] = best[0]                     # elitism: never lose the incumbent
        # fresh trial seeds each generation, so the winner is not one that merely
        # suits a particular servo-offset draw
        # Successive halving. A gait that survives 6 trials often fails 40, so spending
        # the whole budget evenly is wasted: screen the population cheaply, then pay for
        # a real robustness estimate only on the survivors. Seeds are redrawn at every
        # stage, so surviving is not a matter of suiting one draw.
        idx = np.arange(args.pop)
        res = None
        for ntr, keep in ((args.trials, args.pop // 3),
                          (args.trials * 3, args.elite * 2),
                          (args.trials * 7, args.elite)):
            seeds = [0] + list(rng.integers(1, 10 ** 6, ntr - 1))
            res = pool.map(_eval, [(args.family, decode(X[i]), seeds, args.loops)
                                   for i in idx])
            f = np.array([fitness(r) for r in res])
            o = np.argsort(-f)
            if keep >= len(idx):
                idx, res = idx[o], [res[i] for i in o]
                break
            idx, res = idx[o[:keep]], [res[i] for i in o[:keep]]
        fit = np.array([fitness(r) for r in res])
        if fit[0] > best[1]:
            best = (np.clip(X[idx[0]], lo, hi), float(fit[0]), res[0])
        for j, i in enumerate(idx):
            if res[j]['ok'] >= RATE_FLOOR:
                archive.append((np.clip(X[i], lo, hi), res[j]['speed']))
        el = np.clip(X[idx[:args.elite]], lo, hi)
        mu = el.mean(axis=0)
        sd = np.maximum(el.std(axis=0), (hi - lo) * 0.03)   # floor keeps it exploring
        b = res[0]
        print(f'gen {g:2d}  best {b["speed"]:6.1f} u/s  pass {b["ok"]*100:5.1f}%  '
              f'incumbent {best[2]["speed"]:6.1f}  [{time.time()-t0:5.0f}s]', flush=True)
    # The search seeds are re-drawn every generation but a finalist can still have got
    # lucky. Re-run the fastest survivors over many fresh trials and keep the quickest
    # one that is still perfect.
    archive.sort(key=lambda z: -z[1])
    fin, seen = [], set()
    for x, sp in archive:
        k = tuple(np.round(x, 4))
        if k in seen:
            continue
        seen.add(k); fin.append(x)
        if len(fin) >= 30:
            break
    if fin:
        vseeds = [0] + list(rng.integers(1, 10 ** 6, args.verify - 1))
        vres = pool.map(_eval, [(args.family, decode(x), vseeds, args.loops) for x in fin])
        cl = [(x, r) for x, r in zip(fin, vres) if r['ok'] >= RATE_FLOOR]
        print(f'\nverify: {len(cl)}/{len(fin)} finalists hold {RATE_FLOOR*100:.0f}%+ '
              f'over {args.verify} trials')
        if cl:
            x, r = max(cl, key=lambda z: z[1]['speed'])
            best = (x, fitness(r), r)
        else:
            x, r = max(zip(fin, vres), key=lambda z: fitness(z[1]))
            best = (x, fitness(r), r)
    pool.close()
    p = decode(best[0])
    print('\nbest parameters:')
    for k in list(SPACES[args.family]):
        v = p[k]
        print(f'  {k:14s} {v:g}' + ('  (fixed)' if k in fixed else ''))
    print(f'  -> {best[2]["speed"]:.1f} units/s, pass {best[2]["ok"]*100:.0f}%, '
          f'margin {best[2]["margin"]:.2f}')


if __name__ == '__main__':
    main()
