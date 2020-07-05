#!/usr/bin/env python

import yaml
import sys
from matplotlib import pyplot as plt
import numpy as np

if len(sys.argv) < 2:
    print("Please specify a file")
    exit()

anc = {0x674f: 'Anchor01',0x6742:  'Anchor02',0x6a1c: 'Anchor03',0x6755: 'Anchor04',0x6a22: 'Anchor05',0x6705: 'Anchor06'}

filename = sys.argv[1]

with open(filename) as dis_file:
    distance_list = yaml.load(dis_file)

positions = distance_list.keys()
ana_pos = 'back_left'
anchors = distance_list[ana_pos].keys()
duration = len(distance_list[ana_pos][anchors[0]])
for pos in positions:
    print(pos, ": ", len(distance_list[pos][anchors[0]]))
window = 10 # two 0.1 sec
window_arr = range(window)
print(window_arr)
accept_p = 4 # how many uwbs we at least needs to have

def print_dicts(d):

    for pos in d:
        print pos, ":"
        for an in d[pos]:
            print "  ", anc[an], ":", d[pos][an], ""

for t in range(duration-window):
    this_ranges = {}
    pos_achieve = 0
    for pos in positions:
        this_ranges[pos] = {}
        for an in distance_list[pos]:
            count = 0
            range = 0
            # print(window)
            # print(range(2))
            for it in window_arr:
                if distance_list[pos][an][t+it] != 0:
                    range += distance_list[pos][an][t+it]
                    count += 1
            if count != 0:
                range = round(float(range)/count)
                this_ranges[pos][an] = range
        if len(this_ranges[pos].keys()) >= accept_p:
            pos_achieve += 1
    if pos_achieve == len(positions):
        print("Time: ", t, " Enough points: ")
        print_dicts(this_ranges)
        # input()
