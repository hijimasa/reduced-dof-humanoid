"""Which robot the generators build for.

    LME_ROBOT=op3 python3 motion/build_project.py

The generators (build_project.py, ik.py, sim_check.py, static_check.py) hold the
motion *structure*; a profile module in profiles/ holds every number that belongs to
one particular robot -- model path, gait geometry, keyframe durations, searched
get-up poses, servo speed. Adding a robot means adding a profile, not editing a
generator.
"""
import importlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

NAME = os.environ.get('LME_ROBOT', 'microban')


def profile(name=None):
    return importlib.import_module(f'profiles.{name or NAME}')


def constants(name=None):
    """The profile's ALL_CAPS names, ready for `globals().update(constants())`."""
    return {k: v for k, v in vars(profile(name)).items() if k.isupper()}
