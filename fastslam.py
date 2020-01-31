

import numpy as np
from UDPComms import Subscriber, Publisher
import copy


class Gaussian:
    def __init__(self, mean, covariance):
        self.mean = mean
        self.covariance = covariance
        self.normalize()

    def normalize(self):
        self.inverse_covariance = np.linalg.inv(covariance)
        self.normalization = 1/ np.sqrt( (2*np.pi)**self.mean.shape[0] * np.linalg.det(covariance))

    def probability(self, location):
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


class Landmarks(List):
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

    def sensor_update(landmarks -> List[Gaussian]) -> None:

        # TODO
        # associate landmarks/create landmarks/delete landmarks
        # calculate probability of this observation
        # update particle weight with baies
        
        # perform map update
        self.map_update(landmarks)

    def map_update(landmarks -> List[Gaussian]) -> None:
        pass


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
            # particle.move()


    def process_lidar(self):
        scan = self.lidar.get()
        landmarks = find_landmarks(scan)

        for particle in self.particles:
            particles.sensor_update(landmarks)

        #resample particles

    @staticmethod
    def find_landmarks(scan) -> List[Gaussian]:
        pass
