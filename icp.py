import time
import threading

import numpy as np
import networkx as nx
from UDPComms import Subscriber,timeout

from utils import Transform, Robot, PointCloud
from output import Vizualizer


class SLAM:
    def __init__(self):
        self.viz = Vizualizer()

        self.odom  = Subscriber(8810, timeout=0.2)
        self.lidar = Subscriber(8110, timeout=0.1)

        self.robot = Robot()
        self.ploted_robot = Robot()

        self.update_time = time.time()
        self.odom_transform = Transform.fromComponents(0)
        self.odom_transfrom_lock = threading.Lock()

        self.keyframes = []
        self.scan = None #most recent keyframe
        self.lidar_scan = None

        self.running = True
        self.threads = []
        self.threads.append( threading.Thread( target = self.update_odom, daemon = True) )
        self.threads.append( threading.Thread( target = self.update_lidar, daemon = True) )
        for thread in self.threads:
            thread.start()

        self.viz.after(100,self.update_viz)
        self.viz.mainloop()


    def update_viz(self):
        try:
            self.viz.plot_Robot(self.ploted_robot)
            self.viz.plot_Robot(self.robot, c="blue")

            if self.scan is not None:
                self.viz.plot_PointCloud(self.scan)
            if self.lidar_scan is not None:
                self.viz.plot_PointCloud(self.lidar_scan, c="blue")

            self.running = all([thread.is_alive() for thread in self.threads])
        except:
            self.running = False
            raise
        self.viz.after( 100 , self.update_viz)


    def update_odom(self):
        dt = 0.1
        while self.running:
            try:
                da, dy = self.odom.get()['single']['odom']
            except timeout:
                # print("odom timeout")
                continue

            da *= dt
            dy *= dt
            t = Transform.fromOdometry(da, (0,dy))
            with self.odom_transfrom_lock:
                self.odom_transform = t.combine(self.odom_transform)
                self.ploted_robot.drive(t)

            time.sleep(dt)
                


    def update_lidar(self):
        dt = 0.12
        while self.running:
            try:
                scan = self.lidar.get()
            except timeout:
                # print("lidar timeout")
                continue

            pc = PointCloud.fromScan(scan)

            # lidar in robot frame
            pc = pc.move(Transform.fromComponents(0, (-100,0) ))
            pc = pc.move( self.robot.get_transform() )
            pc.location = self.robot.get_transform()

            if len(self.keyframes) == 0:
                self.keyframes.append(pc)
                self.scan = pc
                self.lidar_scan = pc.copy()
                continue

            #hack for now
            self.lidar_scan.points = pc.copy().points
            cloud, transform = self.scan.fitICP(pc)

            robot = self.robot.get_transform().get_components()[1]
            scan  = self.scan.location.get_components()[1]

            if cloud is not None:
                print("robot pos updated")
                self.robot.move(transform)

                if np.linalg.norm(robot - scan) > 500:
                    print("new keyframe")
                    self.scan = pc.move(transform)
                    self.scan.location = self.robot.get_transform()
                    self.keyframes.append( self.scan )

            with self.odom_transfrom_lock:
                self.robot.drive(self.odom_transform)
                self.odom_transform = Transform.fromComponents(0)
                self.ploted_robot.transform = self.robot.get_transform().copy()

            time.sleep(dt)



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


