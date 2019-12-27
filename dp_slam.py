

import numpy as np


class Pose():
    def __init__(self):
        self.pose = np.array([0,0,0])

    @property
    def x(self): return self.pose[0]
    @x.setter
    def x(self, value): self.pose[0] = value 

    @property
    def y(self): return self.pose[1]
    @y.setter
    def y(self, value): self.pose[1] = value 

    @property
    def a(self): return self.pose[2]
    @a.setter
    def a(self, value): self.pose[2] = value 

    def move(dpose):
        pass


class Particle:

    def __init__(self, parent = None):
        self.parent = parent
        self.children = []

        self.onodes = []
        self.pose = parent.pose.copy()

    def prune(self):
        pass

    def colapse(self):
        pass

    def update_odometry(self, delta_pose):
        pass

    def update_lidar(self, scan):
        pass

    def posterior(self, scan):
        pass

    def spawn_child(self):
        child = Particle(self)
        child.pose.move([0,0,0])
        self.children.append(child)
        

    def grid(self,grid):
        grid[self.pose.

class ONode:

    def __init__(self, anode, ancestor):
        self.passes = ancestor.passes
        self.hits = ancestor.hits

        self.ancestor = ancestor
        self.anode = anode

class Grid:

    MM_per_GRID = 30
    GRID_SIZE = 400

    def __init__(self):
        pass
        self.grid = []





class SLAM:
    def __init__(self):
        self.odom =  Subscriber()
        self.lidar = Subscriber()


        self.particle_tree = 
        pass



class Display:
    pass

