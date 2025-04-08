#!/usr/bin/env python3

# requires espeak on the system, on Debian derivatives (including Ubuntu, Raspbian):
# sudo apt install espeak

from argparse import ArgumentParser
from subprocess import Popen
from time import sleep

before = ["ready", "set", "go"]
words = ["end", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine"]

parser = ArgumentParser()
parser.add_argument("-c", "--count", type=int, default=5)
args = parser.parse_args()

if args.count < 1 or args.count > len(words) + 1:
    parser.error("count out of range")

time = args.count
for word in before:
    Popen(["espeak", word])
    print(word)
    sleep(1)
time -= 1

while time >= 0:
    word = words[time]
    Popen(["espeak", word])
    print(word)
    sleep(1)
    time -= 1

