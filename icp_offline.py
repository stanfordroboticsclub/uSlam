

from utils import PointCloud
import json
import argparse
from io import StringIO
from contextlib import redirect_stdout
import numpy as np

import matplotlib.pyplot as plt

pcs = []

with open("data/upstairs.txt") as f:
    for i,line in enumerate(f):
        scan = json.loads(line)
        pc = PointCloud.fromScan(scan)
        pcs.append(pc)

pcs = pcs[::2]
print("loaded {} scans", len(pcs))

output     = [ [None]*len(pcs) for _ in range(len(pcs))]
transforms = [ [None]*len(pcs) for _ in range(len(pcs))]
distances  = [ [None]*len(pcs) for _ in range(len(pcs))]
prints     = [ [None]*len(pcs) for _ in range(len(pcs))]

for i, first in enumerate(pcs):
    print(i)
    for j,second in enumerate(pcs):
        save = StringIO()

        try:
            with redirect_stdout(save):
                cloud, transform = first.fitICP(second)
        except:
            print(save.getvalue())
            raise

        output    [i][j] = 0 if cloud is None else 1
        transforms[i][j] = transform
        distances [i][j] = first.last_matched_distances
        prints    [i][j] = save.getvalue()


def plot_match(x,y):
    bins = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90]
    axs[1].clear()
    axs[1].hist(distances[x][y], bins, density=True)

    axs[2].clear()

    axs[2].plot(pcs[x].points[:,0], pcs[x].points[:,1],".", markersize=2)
    transformed = pcs[y].move(transforms[x][y])
    axs[2].plot(transformed.points[:,0], transformed.points[:,1],".", markersize=2)

    sucess = "sucessfully" if output[x][y] == 1 else "failed"
    fig.suptitle('{} and {} matched {}'.format(x,y, sucess))

    fig.canvas.draw()

scale = 5
fig, axs = plt.subplots(1, 3 ,figsize=(3*scale,scale))
axs[0].matshow(output)
plot_match(0,0)

fig.suptitle('Click Matrix plot to start!')

def hover(event):
    if event.inaxes == axs[0]:
        x = int(event.ydata) #note the direction swap!
        y = int(event.xdata)
        print()
        print(prints[x][y])
        print(x,y)
        print("distances shape", distances[x][y].shape )
        print("distances mean ", np.mean(distances[x][y]) )
        plot_match(x,y)


# fig.canvas.mpl_connect("motion_notify_event", hover)
fig.canvas.mpl_connect("button_press_event", hover)

plt.show()



