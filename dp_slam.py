

import numpy as np


class Pose():
    lidar_offset_forward = 0
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

    @property
    def loc(self):
        return (self.x, self.y)

    def move(self, dpose):
        v_forward, v_right, v_th = dpose
        eff_a = self.a + v_th/2

        # np.rot_matrix( eff_a) @ dpose[:1]

        delta_x = (v_forward * np.cos(eff_a) ) - v_right * math.sin(eff_a))
        delta_y = (v_forward * np.sin(eff_a) ) + v_right * math.cos(eff_a))

        self.x += delta_x;
        self.y += delta_y;
        self.th += v_th;

    def laser(self, dist, angle):
        return (self.x + dist * np.cos(angle + self.a) + self.lidar_offset_forward * np.cos(self.a), 
                self.y + dist * np.sin(angle + self.a) + self.lidar_offset_forward * np.sin(self.a))


class Particle:

    def __init__(self, parent = None):
        self.parent = parent
        self.children = []

        self.onodes = {}
        self.pose = parent.pose.copy()

    def prune(self):
        pass


    def update_odometry(self, delta_pose):
        self.pose.move(delta_pose)
        self.pose.move([0,0,0])

    def update_lidar(self, scan):
        pass

    def posterior(self, scan):
        pass

    def spawn_child(self):
        child = Particle(self)
        child.pose.move([0,0,0])
        self.children.append(child)

    def die(self):
        if len(self.children) == 1:
            pass
            # self = self.children[0]

        
    def grid(self, loc):
        particle = self
        while loc not in particle.onodes.keys():
            particle = particle.parent
            if particle is None:
                return None
        onode = particle.onodes[loc]
        return onode.hits/onode.passes
        

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
        grid_squares = self.GRID_SIZE // self.MM_per_GRID
        self.grid = [[[] for _ in range(grid_squares)] for _ in range(grid_squares)]


    def __getitem__(self, key):
        x, y = key
        return self.grid[x//self.MM_per_GRID][y//self.MM_per_GRID]

    def __setitem__(self, key, value):
        x, y = key
        self.grid[x//self.MM_per_GRID][y//self.MM_per_GRID].append(value)





class SLAM:
    PARTICLE_NUM = 100
    def __init__(self):
        self.odom =  Subscriber()
        self.lidar = Subscriber()

        self.particle_tree = Particle()
        self.particle_list = [self.particle_tree]

    def populate_particles(self):
        while len(self.particle_list) < self.PARTICLE_NUM:
            random.choice(self.particle_list).spawn_child()

    def update_odometry(self, odometry_msg):
        
        delta_pose = np.array(odometry_msg)
        for particle in self.particle_list:
            particle.update_odometry(delta_pose)

    def update(self):
        self.update_odometry()
        
        self.calculate_posteriors()

        self.update_maps()

        self.populate_particles()







class Display:
    pass

