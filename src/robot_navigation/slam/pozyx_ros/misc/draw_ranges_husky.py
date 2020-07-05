#!/usr/bin/env python

# Anchor Number v.s. Anchor Tag ID
# anchor01 = 0x674f
# anchor02 = 0x6742 
# anchor03 = 0x6a1c
# anchor04 = 0x6755
# anchor05 = 0x6a22
# anchor06 = 0x6705
## See this for more https://github.com/ARG-NCTU/subt-system

import yaml
import sys
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.markers as markers
from matplotlib.lines import Line2D
import numpy as np

if len(sys.argv) < 2:
    print("Please specify a file")
    exit()

filename = sys.argv[1]

with open(filename) as dis_file:
    distance_list = yaml.load(dis_file)

dots = {0x674f: 'g.',0x6742:  'b.',0x6a1c: 'r.',0x6755: 'c.',0x6a22: 'm.',0x6705: 'y.'}
pos = {'back_left': 'g.','back_right':  'b.','front_left': 'r.','front_right': 'c.'}
anc = {0x674f: 'Anchor01',0x6742:  'Anchor02',0x6a1c: 'Anchor03',0x6755: 'Anchor04',0x6a22: 'Anchor05',0x6705: 'Anchor06'}
bl_patch = Line2D([0], [0], marker='.', color='w', label='back_left', markerfacecolor='g', markersize=15)
br_patch = Line2D([0], [0], marker='.', color='w', label='back_right', markerfacecolor='b', markersize=15)
fl_patch = Line2D([0], [0], marker='.', color='w', label='front_left', markerfacecolor='r', markersize=15)
fr_patch = Line2D([0], [0], marker='.', color='w', label='front_right', markerfacecolor='c', markersize=15)
patches = [bl_patch, br_patch, fl_patch, fr_patch]
# anc_patches = {0x674f: a1_patch,0x6742: a2_patch,0x6a1c: a3_patch,0x6755: a4_patch,0x6a22: a5_patch,0x6705: a6_patch}
for anchor in distance_list['back_left']:
    count = 0
    for position in distance_list:
        distance_list[position][anchor] = np.array(distance_list[position][anchor], dtype=np.double)
        distance_list[position][anchor][distance_list[position][anchor]==0] = np.nan
        # print(distance_list[position][anchor])
        print(len(distance_list[position][anchor]))
        print("Count: ",str(count), " Color: ", dots[anchor])
        plt.rcParams["figure.figsize"] = (20,6)
        plt.plot(range(len(distance_list[position][anchor])), distance_list[position][anchor], pos[position])
        # plt.legend()
        plt.legend(handles=patches)
        # plt.ylabel('distance in mm')
        # plt.xlabel('Round2    Husky1    ' + position)
        # plt.ylim(bottom=10, top=30000)
        # plt.xticks(np.arange(0, len(distance_list[position][anchor]), step=1000), np.arange(0, len(distance_list[position][anchor])/10., step=100))
        # plt.show()
        count += 1
    # plt.legend()
    plt.ylabel('distance in mm')
    plt.xlabel('Round2    Husky1    ' + anc[anchor])
    plt.ylim(bottom=10, top=30000)
    plt.xticks(np.arange(0, len(distance_list[position][anchor]), step=1000), np.arange(0, len(distance_list[position][anchor])/10., step=100))
    plt.show()