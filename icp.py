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

        self.graph = nx.Graph()
        self.graph_lock = threading.Lock()

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
            # self.viz.plot_Robot(self.ploted_robot, tag="ploted")
            # self.viz.plot_Robot(self.robot, c="blue", tag="main")

            with self.graph_lock:
                for node,data in self.graph.nodes.items():
                    r = Robot()
                    r.transform = data['pose'].copy()
                    pc = data['pc'].copy()
                    if(node not in self.viz.tags.keys()):
                        self.viz.plot_PointCloud(pc, tag=str(pc)+str(node))
                        self.viz.plot_Robot(r, tag=node, c="green")

                for edge, data in self.graph.edges.items():
                    t = str(edge[0])+"_"+str(edge[1])
                    if t not in self.viz.tags:
                        print("plooting edge", edge[0], edge[1])
                        p1 = self.graph.nodes[edge[0]]['pose'].get_components()[1]
                        p2 = self.graph.nodes[edge[1]]['pose'].get_components()[1]
                        self.viz.plot_line(p1, p2, tag=t)

            self.running = all([thread.is_alive() for thread in self.threads])
            if not self.running:
                raise RuntimeError
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
            self.viz.plot_Robot(self.ploted_robot, tag="ploted")

            time.sleep(dt)
                

    def close_keyframes(self):
        MIN_DIST = 300
        MAX_DIST = 800
        robot = self.robot.get_transform().get_components()[1]

        out = []
        for node,data in self.graph.nodes(data=True):
            loc = data['pose'].get_components()[1]
            distance = np.linalg.norm(robot - loc)
            if distance < MAX_DIST:
                out.append( (distance, node,) )

        return sorted(out)


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

            if self.graph.number_of_nodes() == 0:
                if pc.points.shape[0] < 50:
                    continue
                # self.scan = pc
                with self.graph_lock:
                    self.graph.add_node(0, pc = pc.copy(), pose = self.robot.get_transform().copy())

                # self.viz.plot_PointCloud(pc)
                # self.viz.plot_Robot(self.robot, c="green")
                continue

            self.viz.plot_PointCloud(pc, c="blue", tag="current")

            keyframes = self.close_keyframes()

            first = True
            for dist, node in keyframes:
                print("mathcing node", node)
                frame = self.graph.nodes[node]['pc']
                cloud, transform = frame.fitICP(pc)

                if cloud is None:
                    print("match failed")
                    continue

                if first:
                    print("robot pos updated")
                    self.robot.move(transform)
                    if dist < 500:
                        print("closest key frame happy")
                        break

                print("new keyframe")
                scan = pc.move(transform)
                with self.graph_lock:
                    idx = self.graph.number_of_nodes()
                    self.graph.add_node(idx, pc = scan, pose = self.robot.get_transform().copy())
                    self.graph.add_edge(idx, node)

                first = False


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


