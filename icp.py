import tkinter as tk
import math
import time

import numpy as np
from sklearn.neighbors import NearestNeighbors
from UDPComms import Subscriber,timeout


class Transform:
    def __init__(self, matrix):
        self.matrix = matrix

    @classmethod
    def fromOdometry(cls, angle, xy):
        matrix = np.eye(3)
        matrix[0,0] = np.cos(angle); matrix[0,1] =-np.sin(angle)
        matrix[1,0] = np.sin(angle); matrix[1,1] = np.cos(angle)
        matrix[:2,2] = xy

        return cls(matrix)

    @classmethod
    def fromComponents(cls, angle, xy = None):
        if xy == None:
            xy = np.zeros((2))
        else:
            xy = np.array(xy)
        return cls.fromOdometry(np.radians(angle), xy)

    def combine(self, other):
        return Transform(self.matrix @ other.matrix)

    def inv(self):
        R = self.matrix[:2, :2]
        matrix = np.eye(3)
        matrix[:2,:2] = np.linalg.inv(R)
        matrix[:2,2]  = np.linalg.inv(R) @ self.matrix[:2, 2]
        return Transform(matrix)

    def get_components(self):
        x,y = self.matrix[:2,:2] @ np.array([1,0])
        angle = np.arctan2(y,x)
        return (angle, self.matrix[:2, 2])


class Robot:
    def __init__(self, xy = (0,0), angle = 0):
        self.tranform = Transform.fromComponents(angle, xy)

    def drive(self, tranform):
        #local move
        self.tranform = self.tranform.combine(tranform)

    def move(self, tranform):
        #global move
        self.tranform = tranform.combine(self.tranform)

    def get_transform(self):
        return self.tranform

    def get_pose(self):
        pos = np.array([0,0,1])
        head = np.array([0,1,1])

        pos  = self.tranform.matrix @ pos
        head = self.tranform.matrix @ head - pos
        return (pos[:2], head[:2])

class PointCloud:
    def __init__(self, array):
        self.points = array

    def copy(self):
        return PointCloud(self.points.copy())

    @classmethod
    def fromScan(cls, scan):
        # from y axis clockwise
        scan = np.array(scan)
        angles = np.radians(scan[:,1])
        dists = scan[:,2]
        array = np.stack([dists*np.sin(angles), dists*np.cos(angles), np.ones(angles.shape)], axis=-1)
        return cls( array )

    def move(self, tranform):
        # print("matrix", tranform.matrix.shape)
        # print("self", self.points.shape)
        return PointCloud( (tranform.matrix @ self.points.T).T )

    def extend(self, other):
        MIN_DIST = 100

        nbrs = NearestNeighbors(n_neighbors=2).fit(self.points)

        # only middle (high resolution) points are valid to add
        print("other", other.points.shape)
        ranges = (other.points - np.mean(other.points, axis=0))[:, :2]
        ranges = np.sum(ranges**2, axis=-1)**0.5
        # print(ranges)
        points = other.points[ ranges < 2500, :]

        if points.shape[0] == 0:
            return self

        distances, indices = nbrs.kneighbors(points)
        
        # print("distances", distances.shape)
        distances = np.mean(distances, axis=-1)
        matched_other = points[distances > MIN_DIST, :]
        return PointCloud( np.vstack( (self.points, matched_other) ))

    def fitICP(self, other, update_odom):
        # TODO: better way of terminating
        transform = Transform.fromComponents(0)
        for itereation in range(20):
            update_odom()
            aligment = self.AlignSVD(other)
            if aligment is None:
                return None, transform

            angle, xy = aligment.get_components()
            dist = np.sum(xy**2)**0.5

            if( np.abs(angle) > 0.3 or dist > 300 ):
                print("sketchy", itereation, angle, dist)
                return None, transform

            transform = aligment.combine(transform)
            other = other.move(aligment)

            if( angle < 0.001 and dist < 1 ):
                print("done", itereation)
                angle, xy = transform.get_components()
                dist = np.sum(xy**2)**0.5
                print("angle", angle, "Xy", xy)
                if( np.abs(angle) > 0.3 or dist > 300):
                    print("sketchy", itereation, angle)
                    return None, Transform(np.eye(3))
                return other, transform
        else:
            print("convergence failure!")
            return None, transform


    def AlignSVD(self, other):
        # other is the one moving
        MAX_DIST = 300

        # print("self", np.where(np.isnan(self.points)) )
        # print("other", np.where(np.isnan(other.points)) )
        # print("other", other.points )

        # keep around
        nbrs = NearestNeighbors(n_neighbors=1).fit(self.points)
        distances, indices = nbrs.kneighbors(other.points)

        distances = np.squeeze(distances)
        indices = np.squeeze(indices)

        # print("distances:", distances.shape)
        # print("indices:", indices.shape)
        # print("other:", other.points.shape)

        matched_indes = indices[distances <= MAX_DIST]
        matched_other = other.points[distances <= MAX_DIST, :]
        matched_self  = self.points[matched_indes, :]

        # print("matched_self", matched_self.shape)
        # print("matched_other", matched_other.shape)

        if matched_self.shape[0] < 10:
            print("not enough matches")
            return None

        self_mean = np.mean(matched_self, axis=0)
        other_mean = np.mean(matched_other, axis=0)

        matched_self = matched_self- self_mean
        matched_other = matched_other - other_mean

        M = np.dot(matched_other.T,matched_self)
        U,W,V_t = np.linalg.svd(M)

        R = np.dot(V_t.T,U.T)

        #consequence of homogeneous coordinates
        assert R[0,2] == 0
        assert R[1,2] == 0
        assert R[2,2] == 1
        assert R[2,0] == 0
        assert R[2,1] == 0
        
        t = self_mean - other_mean
        R[:2,2] = t[:2]
        
        return Transform(R)


class Vizualizer(tk.Tk):
    def __init__(self, size = 1000, mm_per_pix = 15):
        super().__init__()
        self.SIZE = size
        self.MM_PER_PIX = mm_per_pix

        self.canvas = tk.Canvas(self,width=self.SIZE,height=self.SIZE)
        self.canvas.pack()

        self.robot = []
        self.point_cloud = []
        
    def clear_PointCloud(self):
        for obj in self.point_cloud:
            self.canvas.delete(obj)

    def plot_PointCloud(self, pc, c='#000000', clear=True):
        for x, y,_ in pc.points:
            point = self.create_point(x, y, c=c)
            if clear:
                self.point_cloud.append(point)

    def plot_Robot(self, robot):
        pos, head = robot.get_pose()
        # print("pos", pos)
        # print("head", head)

        head *= 20

        for obj in self.robot:
            self.canvas.delete(obj)

        arrow = self.canvas.create_line(self.SIZE/2 + pos[0]/self.MM_PER_PIX,
                           self.SIZE/2 - pos[1]/self.MM_PER_PIX,
                           self.SIZE/2 + pos[0]/self.MM_PER_PIX + head[0],
                           self.SIZE/2 - pos[1]/self.MM_PER_PIX - head[1],
                           arrow=tk.LAST)

        oval = self.canvas.create_oval(self.SIZE/2+5 + pos[0]/self.MM_PER_PIX, 
                                self.SIZE/2+5 - pos[1]/self.MM_PER_PIX,
                                self.SIZE/2-5 + pos[0]/self.MM_PER_PIX,
                                self.SIZE/2-5 - pos[1]/self.MM_PER_PIX,
                                fill = '#FF0000')

        self.robot = [arrow,oval]

    def create_point(self,x,y, c = '#000000', w= 1):
        return self.canvas.create_oval(self.SIZE/2 + x/self.MM_PER_PIX,
                                self.SIZE/2 - y/self.MM_PER_PIX,
                                self.SIZE/2 + x/self.MM_PER_PIX,
                                self.SIZE/2 - y/self.MM_PER_PIX, width = w, fill = c, outline = c)




class SLAM:
    def __init__(self):
        self.viz = Vizualizer()

        self.odom  = Subscriber(8810, timeout=0.2)
        self.lidar = Subscriber(8110, timeout=0.1)

        self.robot = Robot()
        self.update_time = time.time()
        self.last_odom = time.time()

        self.scan = None

        self.odom_acc = Transform.fromComponents(0)

        self.viz.after(100,self.update)
        self.viz.mainloop()


    def update(self):
        self.update_time = time.time()

        self.update_odom()

        self.robot.drive(self.odom_acc)
        self.odom_acc = Transform.fromComponents(0)

        self.viz.plot_Robot(self.robot)

        self.update_lidar()
        print('update lidar',time.time() - self.update_time)

        loop_time = 1000 * (time.time() - self.update_time)
        print('full loop',time.time() - self.update_time)
        self.update_odom()

        self.viz.after( int(max(100 - loop_time, 0)) , self.update)

    def update_odom(self):
        print("update odom called")
        dt = time.time() - self.last_odom
        # if( dt < 0.05):
        #     return

        try:
            da, dy = self.odom.get()['single']['odom']
        except timeout:
            print("odom timeout")
            return
        print("odom dt", dt)
        self.last_odom = time.time()

        da *= dt
        dy *= dt

        self.odom_acc =  Transform.fromOdometry(da, (0,dy)).combine(self.odom_acc)

    def update_lidar(self):
        try:
            scan = self.lidar.get()
            # print("scan", scan)
            pc = PointCloud.fromScan(scan)

            # lidar in robot frame
            pc = pc.move(Transform.fromComponents(0, (-100,0) ))
            pc = pc.move( self.robot.get_transform() )

            if(self.scan == None):
                self.scan = pc
                self.viz.plot_PointCloud(self.scan, clear = False)
            else:
                self.viz.clear_PointCloud()
                # self.viz.plot_PointCloud(self.scan)
                self.viz.plot_PointCloud(pc, c="blue")

                cloud, transform = self.scan.fitICP(pc, lambda :self.update_odom() )
                print('update lidar',time.time() - self.update_time)
                if cloud is not None:
                    self.robot.move(transform)
                    # self.viz.plot_PointCloud(cloud, c="red")
                    self.scan = self.scan.extend( cloud )
                    self.viz.plot_PointCloud( cloud, clear = False)
                    print('update plot',time.time() - self.update_time)

        except timeout:
            print("lidar timeout")
            pass


if __name__ == "__main__":
    s = SLAM()

    # v = Vizualizer()
    # s1 = PointCloud.fromScan(scan1).move(Transform.fromComponents(0, (400,0)))
    # s2 = PointCloud.fromScan(scan2).move(Transform.fromComponents(15, (400,0)))

    # v.plot_PointCloud(s1)
    # v.plot_PointCloud(s2, c="blue")

    # s3, transform = s1.fitICP(s2)
    # # v.plot_PointCloud(s3, c="green")

    # s4 = s2.move(transform)
    # v.plot_PointCloud(s4, c="green")

    # v.mainloop()


