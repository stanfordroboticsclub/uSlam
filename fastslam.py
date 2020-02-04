

import numpy as np
from UDPComms import Subscriber, Publisher
import copy

from typing import List

Bearing = List[float]
Position = type(np.array([0,0]))


class Gaussian:
    def __init__(self, mean, covariance):
        self.mean = mean
        self.covariance = covariance
        self.normalize()
        self.existing = 1

    def normalize(self):
        self.inverse_covariance = np.linalg.inv(covariance)
        self.normalization = 1/ np.sqrt( (2*np.pi)**self.mean.shape[0] * np.linalg.det(covariance))

    def probability(self, location: Position) -> float:
        offset = location-self.mean 
        return self.normalization * np.exp( -0.5 * offset.T @ self.inverse_covariance @ offset)

    @classmethod
    def fromAngle(cls, mean, angle, stds):
        pass

    def copy(self):
        return copy.deepcopy(self)

class Kalman(Gaussian):

    def update(self, observation: Gaussian) -> None:
        kalman = self.covariance @ np.linalg( self.covariance + observation.covariance)
        self.mean = self.mean + kalman @ (observation.mean + self.mean)
        self.covariance = self.covariance - kalman @ self.covariance
        self.normalize()


class Landmarks(list):
    pass
    #TODO: update to reference past particles


class Particle:
    def __init__(self):
        self.landmarks = Landmarks()
        self.weight = 1

        self.position = np.array([0,0,0])

    def move(self, delta_xy, delta_r):
        s = np.sin(self.position[-1] + rot/dt/2)
        c = np.cos(self.position[-1] + rot/dt/2)

        R = np.array([ [c,-s], [s, c] ])
        self.position[:2] += (R @ delta_xy)
        self.position[-1] += delta_r

    def sensor_update(landmarks -> List[Bearing]) -> None:

        # TODO
        # get landmark sensor distribution
        # associate landmarks/create landmarks/delete landmarks
        # calculate probability of this observation
        # update particle weight with baies
        
        # perform map update
        self.map_update(landmarks)

    def map_update(landmarks -> List[Gaussian]) -> None:
        pass

    def compute_landmarks(landmarks: List[Bearing]) -> List[Gaussian]:
        # normally done with Jacobian. Here we eyeball it

        radial_std = 1
        tangential_std = 1

        out = []
        for distance,angle in landmarks:
            alpha = self.position[-1] + angle
            c = np.cos(alpha)
            s = np.sin(alpha)
            location = self.position[:2] + dist * np.array([c, s])
            T =  np.array([ [c,-s],[s,c] ]) @ np.diag([radial_std, tangential_std])
            covar = T @ T.T
            out.append(Gaussian(location, covar))

        return out



class SLAM:
    pass

    def __init__(self):
        self.particle_num = 100
        self.particles = [ Particle() for _ in range(self.particle_num) ]

        self.odom = Subscriber()
        self.lidar = Subscriber()


    def loop(self):
        pass

    def process_odometry(self):
        movement = self.odom.get()
        delta_xy = movement["xy"]/dt
        delta_r  = movement["angle"]/dt

        for particle in self.particles:
            particle.move(delta_xy, delta_r)
            angle_var = np.degrees(5) * np.random.randn()
            xy_var = [0.01 * np.random.randn(), 0]
            particle.move(xy_var, angle_var)
            # particle.move()


    def process_lidar(self):
        scan = self.lidar.get()
        landmarks = find_landmarks(scan)

        for particle in self.particles:
            particles.sensor_update(landmarks)

        #resample particles

    @staticmethod
    def find_landmarks(scan) -> List[Bearing]:

        xs = []
        ys = []

        img = np.zeros((500,500))

        for _, angle, dist in scan:
            a = math.radians(angle)
            x = math.sin(a) * dist
            y = math.cos(a) * dist
            img[x,y] = 1

        img



