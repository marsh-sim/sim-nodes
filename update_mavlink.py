#!/usr/bin/env python3

"""
Generates the Python library from newest definitions at https://github.com/marsh-sim/mavlink/tree/dialect

Doesn't accept any arguments by design.

Based on:
https://github.com/marsh-sim/marsh-sim.github.io/blob/main/update_mavlink_tables.py
https://github.com/marsh-sim/marsh-manager/blob/main/scripts/update_mavlink.py
"""

from os import path as p
import subprocess
import sys

# allow running from anywhere
root_p = p.dirname(__file__)

# clone the mavlink repository
repo_p = p.join(root_p, "mavlink_repo")
if not p.exists(repo_p):
    subprocess.check_call(
        [
            "git",
            "clone",
            "--recursive",
            "https://github.com/marsh-sim/mavlink.git",
            repo_p,
        ],
        cwd=root_p,
    )

# update to latest commit on the dialect branch
subprocess.check_call(["git", "fetch"], cwd=repo_p)
subprocess.check_call(["git", "checkout", "origin/dialect"], cwd=repo_p)

# add repo path to import pymavlink from there
sys.path.insert(1, root_p)
# fmt: off - don't move import to top of file
from mavlink_repo.pymavlink.generator import mavgen
from mavlink_repo.pymavlink.generator.mavparse import PROTOCOL_2_0

# fmt: on

# generate Python library with all definitions
definitions_path = p.join(repo_p, "message_definitions", "v1.0")
dialect_file = "all.xml"
out_path = p.join(root_p, "mavlink_all")

opts = mavgen.Opts(
    out_path, wire_protocol=PROTOCOL_2_0, language="Python3", validate=False
)
mavgen.mavgen(opts, [p.join(definitions_path, dialect_file)])
