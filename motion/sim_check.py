#!/usr/bin/env python3
"""Replay an LME project in MuJoCo and report whether each action actually works.

Reproduces LME's playback (per-joint easing over `frames`/`fps` seconds, plus the
joint speed limiter) and drives position actuators with the interpolated targets.

A single deterministic run is not a useful verdict: a marginal gait flips between
standing and falling on a 1e-4 deg change. So each action is run `--trials` times
with a different servo-offset / initial-tilt draw, and the score is the survival
rate. Anything below 100% is a gait that would be luck on real hardware.
"""
import argparse
import math
import os
import sys
import re
import xml.etree.ElementTree as ET

import numpy as np
import mujoco

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_profile import profile

_P = profile()
MJCF = os.path.join(REPO, _P.MODEL.lstrip('./'))
PROJECT = os.path.join(REPO, _P.OUT)

KP = _P.SIM_KP
FORCE = _P.SIM_FORCE
MAX_SPEED = _P.SIM_MAX_SPEED   # rad/s, matching the project's JointSettings

# Per-trial disturbances, standing in for servo zero error and an imperfect floor.
OFFSET_SD = 0.3      # deg, constant per joint per trial
TILT_SD = 1.0        # deg, initial trunk roll/pitch
# Studio does not hold its nominal frame rate: it plays a motion at whatever rate it can
# render, so the real playback speed drifts. A motion tuned at exactly 1.0 has no reason
# to survive that, and measured, this one did not -- the walk was 8/8 at 1.0 and 0/8 at
# 0.7. Drawing a speed per trial makes the search pay for the whole band instead.
SCALE_JITTER = 0.0   # trial scale ~ U(1 - j, 1 + j); 0 disables

# Actions that do not start from a standing robot: trunk pitch at t=0, in degrees.
# +90 lays the chest on the floor, -90 lays it on its back.
START_PITCH = {'standup_face_down': 90.0, 'standup_face_up': -90.0}

EASINGS = {
    'linear': lambda x: x,
    'easeInSine': lambda x: 1 - math.cos(x * math.pi / 2),
    'easeOutSine': lambda x: math.sin(x * math.pi / 2),
    'easeInOutSine': lambda x: -(math.cos(math.pi * x) - 1) / 2,
    'easeInQuad': lambda x: x * x,
    'easeOutQuad': lambda x: 1 - (1 - x) ** 2,
    'easeInOutQuad': lambda x: 2 * x * x if x < 0.5 else 1 - ((-2 * x + 2) ** 2) / 2,
}


def ease(name, x):
    key = name.split(':', 1)[1].strip() if ':' in name else name
    return EASINGS.get(key, EASINGS['linear'])(x)


# Overrides for asking "how much does this motion depend on the physics being exactly
# what this machine produces". A motion that only works at one floor friction, or one
# timestep, is a motion that will not survive being opened on someone else's computer.
FLOOR_FRICTION = '0.08 0.003 0.0001'
TIMESTEP = None


def build_scene(path):
    """robot.xml + floor, with the torque motors swapped for position servos.

    Some models collide only through explicit <pair> elements -- ToddlerBot ships with
    contype/conaffinity 0 on every geom and a hand-picked pair list, so a floor dropped
    in next to it is a floor the robot falls straight through. FLOOR_PAIRS in the
    profile names the geoms that should touch the ground; models that collide the
    ordinary way leave it empty and nothing is emitted.
    """
    src = open(MJCF, encoding='utf-8').read()
    # The scene is written somewhere else, so a relative meshdir stops resolving. Make
    # it absolute against the model's own directory, whatever it happens to be called.
    src = re.sub(r'meshdir="(?!/)([^"]*)"',
                 lambda mo: f'meshdir="{os.path.join(os.path.dirname(MJCF), mo.group(1))}"', src)
    src = re.sub(r'<motor class="xl330" name="([^"]+)" joint="([^"]+)"/>',
                 rf'<position class="xl330" name="\1" joint="\2" kp="{KP}" dampratio="1" '
                 rf'forcerange="-{FORCE} {FORCE}"/>', src)
    # Match the scene MuJoCo Studio actually runs (log/_active_scene.xml): elliptic
    # friction cone, impratio 10 and the implicitfast integrator. These change contact
    # behaviour enough that a gait tuned against MuJoCo's defaults can be noticeably
    # less robust in Studio, so verify against the same physics.
    ts = f' timestep="{TIMESTEP}"' if TIMESTEP else ''
    opt = ('<option solver="Newton" cone="elliptic" integrator="implicitfast" '
           f'impratio="10"{ts}/>\n')
    src = src.replace('<mujoco model=', opt + '<mujoco model=') if False else src
    if '<option' not in src:
        src = src.replace('</compiler>', '</compiler>\n  ' + opt, 1)
        if '<option' not in src:
            i = src.index('>', src.index('<compiler')) + 1
            src = src[:i] + '\n  ' + opt + src[i:]
    pairs = getattr(_P, 'FLOOR_PAIRS', ())
    # (emitted below, once the floor geom exists)
    # Copied verbatim from the floor Studio generates (log/_active_scene.xml). solref
    # 0.005 with a 0.002 timestep is only 2.5x the step, i.e. a stiff contact right at
    # the stability limit -- loaded parts sink into it. Leaving this out made the
    # harness far more forgiving than Studio.
    floor = ('<geom name="floor" type="box" size="15 15 0.002" pos="0 0 -0.001" '
             f'friction="{FLOOR_FRICTION}" condim="3" solref="0.005 1" '
             'solimp="0.9 0.95 0.001 0.5 2"/>\n'
             '<light pos="0 0 3" dir="0 0 -1" directional="true"/>\n  </worldbody>')
    src = src.replace('</worldbody>', floor, 1)
    if pairs:
        block = '\n'.join(f'    <pair geom1="floor" geom2="{g}"/>' for g in pairs)
        if '<contact>' in src:
            src = src.replace('<contact>', '<contact>\n' + block, 1)
        else:
            src = src.replace('</mujoco>', f'  <contact>\n{block}\n  </contact>\n</mujoco>', 1)
    open(path, 'w', encoding='utf-8').write(src)
    return path


def load_actions(project):
    root = ET.parse(project).getroot()
    out = {}
    for a in root.find('Actions'):
        nodes = {n.get('id'): n for n in a.find('Nodes')}
        edges = [(e.get('from'), e.get('to'), int(e.get('from_port', 0)))
                 for e in a.find('Edges')]
        out[a.get('title')] = (nodes, edges, a.find('Playback'))
    return out


def expand(action, loops):
    """Follow the graph into a flat pose list; branch 'then' is taken `loops` times."""
    nodes, edges, pb = action
    seq, taken = [], {}
    cur, guard = pb.get('start_node_id'), 0
    while cur and guard < 4000:
        guard += 1
        n = nodes.get(cur)
        if n is None or n.get('type') == 'jump':
            break
        if n.get('type') == 'pose':
            seq.append((n.get('name'), int(n.get('frames', 10)),
                        {x.get('joint'): float(x.get('value')) for x in n.find('Angles')},
                        {x.get('joint'): x.get('value') for x in n.find('Easings')}))
            port = 0
        else:
            taken[cur] = taken.get(cur, 0) + 1
            port = 1 if taken[cur] <= loops else 0
        nxt = [t for f, t, p in edges if f == cur and p == port]
        cur = nxt[0] if nxt else None
    return seq, float(pb.get('fps', 100))


class Sim:
    def __init__(self, scene, scale=1.0):
        self.scale = scale
        self.m = mujoco.MjModel.from_xml_path(scene)
        self.d = mujoco.MjData(self.m)
        self.names = [mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                      for i in range(self.m.nu)]
        self.qadr = [self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                     for n in self.names]
        self.sfoot = {s: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, f'{s}_foot')
                      for s in ('right', 'left')}
        # Passive joints slaved to actuated ones by a linkage (parallel-link legs). The
        # physics keeps the loops closed once running, but a start pose that writes the
        # actuated joints and leaves the passive ones at zero begins with the loops torn
        # open, and the constraint solver throws the robot over before the first frame.
        qa = lambda n: self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)]
        self.coupling = [(qa(pj), [(qa(a), k) for a, k in terms])
                         for pj, terms in getattr(_P, 'PASSIVE_COUPLING', {}).items()]

    def _couple(self):
        for pa, terms in self.coupling:
            self.d.qpos[pa] = sum(k * self.d.qpos[a] for a, k in terms)

    def _settle(self, angles, tilt, pitch0=0.0):
        mujoco.mj_resetData(self.m, self.d)
        # A project need not pose every actuator: ToddlerBot has wrists and a waist that
        # nothing in this repo drives. Anything the project is silent about is held at 0.
        q = np.array([angles.get(n, 0.0) for n in self.names])
        for a, v in zip(self.qadr, q):
            self.d.qpos[a] = np.deg2rad(v)
        self._couple()
        self.d.ctrl[:] = np.deg2rad(q)
        r, p = np.deg2rad(tilt)
        p += math.radians(pitch0)
        self.d.qpos[3:7] = [math.cos(r / 2) * math.cos(p / 2), math.sin(r / 2) * math.cos(p / 2),
                            math.cos(r / 2) * math.sin(p / 2), 0.0]
        mujoco.mj_forward(self.m, self.d)
        if pitch0:
            # lying down: drop from above and let it come to rest on whatever touches first
            self.d.qpos[2] += 0.12 - min(self.d.site_xpos[i][2] for i in self.sfoot.values())
            seconds = 2.0
        else:
            # shift, do not assign: Studio's play scene spawns the body at a computed
            # height, so qpos[2] is not zero-referenced there
            self.d.qpos[2] += -min(self.d.site_xpos[i][2] for i in self.sfoot.values()) + 0.001
            seconds = 1.0
        mujoco.mj_forward(self.m, self.d)
        for _ in range(int(seconds / self.m.opt.timestep)):
            mujoco.mj_step(self.m, self.d)
        return q

    def _trunk(self):
        w, x, y, z = self.d.qpos[3:7]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return self.d.qpos[:3].copy(), math.degrees(yaw), self.d.xmat[1].reshape(3, 3)[2, 2]

    def standing_comz(self, angles):
        """Centre-of-mass height, in mm, once settled standing in `angles`."""
        self._settle(angles, (0.0, 0.0))
        return float(self.d.subtree_com[0][2]) * 1000

    def play(self, seq, fps, offsets=None, tilt=(0.0, 0.0), settle_after=0.6, pitch0=0.0,
             min_comz=0.0, latency=0, dropout=0.0, rng=None):
        off = np.zeros(len(self.names)) if offsets is None else offsets
        cmd = self._settle(seq[0][2], tilt, pitch0)
        p0, yaw0, _ = self._trunk()
        dt = self.m.opt.timestep
        steps = max(1, round(1.0 / fps / dt))
        step_lim = math.degrees(MAX_SPEED) / fps
        rec = dict(min_up=1.0, sat=0, ticks=0, fell_at=None)
        prev, last, t_s = cmd.copy(), cmd.copy(), 0.0
        # LME does not drive the simulator directly: it writes a Meridim frame to Valkey
        # every tick and MuJoCo Studio reads it on its own clock. That path has latency
        # and drops frames when the machine is busy, so the robot holds a stale target.
        # A slow gait never notices; a fast one is a different question, which is the
        # whole point of modelling it here.
        pipe = [cmd.copy() for _ in range(latency + 1)]
        grace = 0.0 if not pitch0 else 1e9   # a get-up starts down; only the end matters
        for name, frames, angles, easings in seq[1:]:
            tgt = np.array([angles.get(n, 0.0) for n in self.names])
            for f in range(1, max(1, int(round(frames * self.scale))) + 1):
                u = f / max(1, int(round(frames * self.scale)))
                for i, n in enumerate(self.names):
                    cmd[i] = prev[i] + (tgt[i] - prev[i]) * ease(easings.get(n, '0: linear'), u)
                cmd = np.clip(cmd, last - step_lim, last + step_lim)
                last = cmd.copy()
                if dropout and rng is not None and rng.random() < dropout:
                    pipe.append(pipe[-1].copy())        # frame lost: hold the last target
                else:
                    pipe.append(cmd.copy())
                self.d.ctrl[:] = np.deg2rad(pipe.pop(0) + off)
                for _ in range(steps):
                    mujoco.mj_step(self.m, self.d)
                    rec['sat'] += int(np.any(np.abs(self.d.actuator_force) >= FORCE * 0.999))
                    rec['ticks'] += 1
                _, _, up = self._trunk()
                rec['min_up'] = min(rec['min_up'], up)
                t_s += 1.0 / fps
                if up < 0.4 and rec['fell_at'] is None and t_s > grace:
                    rec['fell_at'] = (t_s, name)
            prev = tgt.copy()
        for _ in range(int(settle_after / dt)):
            mujoco.mj_step(self.m, self.d)
        p1, yaw1, up1 = self._trunk()
        c, s = math.cos(-math.radians(yaw0)), math.sin(-math.radians(yaw0))
        dp = p1 - p0
        rec.update(dx=c * dp[0] - s * dp[1], dy=s * dp[0] + c * dp[1],
                   dyaw=(yaw1 - yaw0 + 180) % 360 - 180, up_end=up1, duration=t_s)
        # An upright trunk is not the same as standing: a robot that ends the get-up on
        # its knees has up ~= 1 and would pass. So the centre of mass has to have got
        # back to (most of) the height the final pose is worth.
        rec['comz'] = float(self.d.subtree_com[0][2]) * 1000
        rec['ok'] = rec['fell_at'] is None and up1 > 0.85 and rec['comz'] >= min_comz
        return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--project', default=PROJECT)
    ap.add_argument('--loops', type=int, default=3, help='branch "then" count (cycles = loops+1)')
    ap.add_argument('--trials', type=int, default=8)
    ap.add_argument('--scale', type=float, default=1.0, help='multiply every frame count')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--floor-friction', default=None,
                    help='replace the floor\'s "slide spin roll" friction. Studio '
                         'hardcodes 0.08 0.003 0.0001; use this to find out how much '
                         'of the margin depends on that exact number')
    ap.add_argument('--timestep', default=None)
    ap.add_argument('--scale-jitter', type=float, default=None,
                    help='draw each trial\'s playback speed from U(1-j, 1+j) instead of '
                         'running them all at exactly the nominal rate')
    ap.add_argument('--stress', default='',
                    help='"offset_sd,tilt_sd" in degrees, replacing the nominal 0.3,1.0. '
                         'The nominal level is what a motion is tuned against; this is '
                         'what tells you how much margin it has left over.')
    ap.add_argument('--latency', type=int, default=0,
                    help='frames of delay between LME writing a target and the sim '
                         'seeing it (the Valkey hop)')
    ap.add_argument('--dropout', type=float, default=0.0,
                    help='probability that a frame is lost and the last target is held')
    ap.add_argument('--scene', default=None,
                    help='use an existing scene instead of building one, e.g. the scene '
                         'MuJoCo Studio actually runs (log/_active_scene.xml). Studio\'s '
                         'scene differs: elliptic cone, impratio=10, implicitfast, and a '
                         'low-friction floor, so a motion verified here can still fail there')
    args = ap.parse_args()

    global OFFSET_SD, TILT_SD, FLOOR_FRICTION, TIMESTEP, SCALE_JITTER
    if args.floor_friction:
        FLOOR_FRICTION = args.floor_friction
    if args.timestep:
        TIMESTEP = args.timestep
    if args.stress:
        OFFSET_SD, TILT_SD = (float(v) for v in args.stress.split(','))
    if args.scale_jitter is not None:
        SCALE_JITTER = args.scale_jitter
    # per-process scene file: two sim_checks running at once used to overwrite each other's
    # scene and silently evaluate the wrong robot
    import tempfile
    scene = os.path.join(tempfile.gettempdir(), f'simcheck_{os.getpid()}.xml')
    os.makedirs(os.path.dirname(scene), exist_ok=True)
    sim = Sim(args.scene or build_scene(scene), args.scale)
    actions = load_actions(args.project)

    print(f'{"action":24s} {"pass":>7s} {"dx mm":>8s} {"dy mm":>8s} {"dyaw":>7s} '
          f'{"t s":>5s} {"sat%":>5s}  worst failure')
    for title, act in actions.items():
        if (args.only and title not in args.only) or title == 'Base':
            continue
        seq, fps = expand(act, args.loops)
        if len(seq) < 2:
            continue
        rng = np.random.default_rng(args.seed)
        floor_comz = 0.80 * sim.standing_comz(seq[-1][2])
        runs = []
        for i in range(args.trials):
            off = np.zeros(len(sim.names)) if i == 0 else rng.normal(0, OFFSET_SD, len(sim.names))
            tilt = (0.0, 0.0) if i == 0 else tuple(rng.normal(0, TILT_SD, 2))
            # Spread deterministically across the band rather than sampling it. A tuner
            # runs 5 or 6 trials per candidate, and 6 uniform draws leave the slow end
            # uncovered about as often as not -- which is the end that kills a gait.
            if SCALE_JITTER and i > 0 and args.trials > 1:
                f = (i - 1) / max(1, args.trials - 2)
                sim.scale = args.scale * (1.0 - SCALE_JITTER + 2 * SCALE_JITTER * f)
            else:
                sim.scale = args.scale
            runs.append(sim.play(seq, fps, off, tilt, pitch0=START_PITCH.get(title, 0.0),
                                 min_comz=floor_comz, latency=args.latency,
                                 dropout=args.dropout, rng=rng))
        good = [r for r in runs if r['ok']]
        bad = [r for r in runs if not r['ok']]
        n = len(runs)
        ref = good or runs
        dx = np.mean([r['dx'] for r in ref]) * 1000
        dy = np.mean([r['dy'] for r in ref]) * 1000
        dyaw = np.mean([r['dyaw'] for r in ref])
        worst = ''
        if bad:
            b = min(bad, key=lambda r: r['fell_at'][0] if r['fell_at'] else 9e9)
            worst = (f"fell {b['fell_at'][0]:.1f}s at {b['fell_at'][1]}" if b['fell_at']
                     else f"ended up={b['up_end']:.2f} comZ={b['comz']:.0f}mm")
        print(f'{title:24s} {len(good):3d}/{n:<3d} {dx:8.1f} {dy:8.1f} {dyaw:7.1f} '
              f'{runs[0]["duration"]:5.1f} {100*runs[0]["sat"]/max(1,runs[0]["ticks"]):5.1f}  {worst}')
    print(f'\n{args.trials} trials/action: trial 0 is nominal, the rest add '
          f'N(0,{OFFSET_SD}) deg servo offsets and N(0,{TILT_SD}) deg initial trunk tilt'
          + (f', playback speed U({1-SCALE_JITTER:.2f},{1+SCALE_JITTER:.2f}).'
             if SCALE_JITTER else '.'))
    print('dx/dy in the robot frame (+x forward, +y left); dyaw over the whole action.')
    print(f'sat% = ticks with at least one servo at the {FORCE} Nm ceiling. With kp={KP:g} '
          f'a ~0.7 deg\n       tracking error already saturates, so treat it as effort, not as a fault.')


if __name__ == '__main__':
    main()
