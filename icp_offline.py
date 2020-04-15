

from utils import PointCloud
import json
import argparse

import numpy as np

import matplotlib.pyplot as plt

pcs = []

with open("data/upstairs.txt") as f:
    for i,line in enumerate(f):
        scan = json.loads(line)
        pc = PointCloud.fromScan(scan)
        pcs.append(pc)

pcs = pcs[:50]
print("loaded {} scans", len(pcs))

output     = [ [None]*len(pcs) for _ in range(len(pcs))]
transforms = [ [None]*len(pcs) for _ in range(len(pcs))]
distances  = [ [None]*len(pcs) for _ in range(len(pcs))]

for i, first in enumerate(pcs):
    print("\n\n\n")
    print(i)
    for j,second in enumerate(pcs):
        cloud, transform = first.fitICP(second)

        output    [i][j] = 0 if cloud is None else 1
        transforms[i][j] = transform
        distances [i][j] = first.last_matched_distances


def plot_match(x,y):
    bins = [0, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90]
    axs[1].clear()
    axs[1].hist(distances[x][y], bins, density=True)

    axs[2].clear()

    axs[2].plot(pcs[x].points[:,0], pcs[x].points[:,1],".", markersize=2)
    transformed = pcs[y].move(transforms[x][y])
    axs[2].plot(transformed.points[:,0], transformed.points[:,1],".", markersize=2)

    fig.canvas.draw()

scale = 7
fig, axs = plt.subplots(1, 3 ,figsize=(3*scale,scale))
axs[0].matshow(output)
plot_match(0,0)

def hover(event):
    if event.inaxes == axs[0]:
        x = int(event.xdata)
        y = int(event.ydata)
        print()
        print(x,y)
        print("distances shape", distances[x][y].shape )
        print("distances mean ", np.mean(distances[x][y]) )
        plot_match(x,y)


# fig.canvas.mpl_connect("motion_notify_event", hover)
fig.canvas.mpl_connect("button_press_event", hover)

plt.show()



