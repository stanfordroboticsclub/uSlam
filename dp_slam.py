

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

        if parent is None:
            self.pose = Pose()
            self.probability = 1
        else:
            self.pose = parent.pose.copy()
            self.probability = parent.probability


    def prune(self, child):
        # TODO: move to sets?
        idx = self.children.index(child)
        del self.children[idx]

        if len(self.children) == 0:
            self.die()

        elif len(self.children) == 1:
            kid = self.children[0]
            self.pose = kid.pose
            self.children = kid.children
            # TODO operation unsopprted: manual code needed
            self.onodes = self.onodes + kid.onodes

    def die(self):
        for key,value in self.onodes.items():
            pass
        if self.parent == None:
            print("killing top level ancestor")

        self.parent.prune(child)

    def spawn_child(self):
        child = Particle(self)
        self.children.append(child)
        return child

    def update_odometry(self, delta_pose):
        self.pose.move(delta_pose)
        # TODO: should we update the particle prob based on this?
        child.pose.move( np.array([10,5,0.3]) * np.random.randn(3) )

    def update_lidar(self, scan):

        for dist, angle in scan:
            loc = self.pose.lidar(dist,angle)
            np.log(self.grid(loc))


    def posterior(self, scan):
        pass

        # p(m|L) = P(L|m)p(m) / p(l)

        for dist, angle in scan:
            loc = self.pose.lidar(dist,angle)
            np.log(self.grid(loc))

        if self.probability < 0.01:
            return False




        
    def grid(self, loc):
        loc = grid.round(loc)

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

    def round(self,key):
        x, y = key
        return (x//self.MM_per_GRID, y//MM_per_GRID)

    def visited_squares(self, robot, point):
        pass





class SLAM:
    PARTICLE_NUM = 100
    def __init__(self):
        self.odom =  Subscriber()
        self.lidar = Subscriber()

        self.particle_tree = Particle()
        self.active_particles = [self.particle_tree]

        self.grid = Grid()

    def sample_particles(self):
        self.active_particles = []
        for _ in range(self.PARTICLE_NUM):
            new = random.choice(self.active_particles).spawn_child()
            self.active_particles.append(new)

    def update_odometry(self, odometry_msg):
        delta_pose = np.array(odometry_msg)
        for particle in self.active_particles:
            particle.update_odometry(delta_pose)

    def update_lidar(self, scan):
        pass
        for particle in self.active_particles:
            particle.posterior(scan)

    def update(self):
        self.update_odometry(odom)

        self.update_lidar(scan)

        self.sample_particles()




class Display:
    pass



if __name__ == "__main__":
    pass
