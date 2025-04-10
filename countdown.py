#!/usr/bin/env python3

"""
Script for reading out a countdown each second.
Requires `espeak` command.
"""

from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
from subprocess import Popen
from time import sleep

before = ["ready", "set", "go"]
words = ["end", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten"]

parser = ArgumentParser(description=__doc__, formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("-c", "--count", type=int, help="seconds to move between states", default=5)
parser.add_argument("--no-ready", action="store_false", dest="ready", help="say 'ready, set, go' before countdown")

args = parser.parse_args()

if args.count < 1 or args.count > len(words):
    parser.error("count out of range")
time: int = args.count

if args.ready:
    for word in before:
        Popen(["espeak", word])
        print(word)
        sleep(1)
    time -= 1

while time >= 0:
    word: str = words[time]
    Popen(["espeak", word])  # run it independently to not affect the timing
    print(word)
    sleep(1)
    time -= 1
