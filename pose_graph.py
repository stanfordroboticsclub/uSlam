
import networkx as nx
import numpy as np
import json

from utils import Transform, PointCloud

# NODE has:
    # ID
    # Pose
    # PC

# EDGE has:
    # ID pair
    # Transform

class PoseGraph:

    def __init__(self, graph = None):
        if graph == None:
            graph = nx.DiGraph()

        self.graph = graph

    def save(self, filename):
        output = {}
        output["nodes"] = {}
        for node, pose, pc in self.get_nodes():

            json_pose = None if pose is None else pose.toJSON() 
            json_pc = None if pc is None else pc.toJSON() 
            output["nodes"][node] = {"pose":  json_pose,
                                     "pc": json_pc , "edges": {}}

        output["edges"] = {}
        for (x,y), transform in self.get_edges():
            output["nodes"][x]["edges"][y] = {"transform": transform.toJSON() }

        with open(filename, "w") as f:
            f.write(json.dumps(output))

    @classmethod
    def load(cls, filename):
        with open(filename, "r") as f:
            output = json.loads(f.read())
        
        graph = nx.DiGraph()

        for node,data in output['nodes'].items():
            graph.add_node(node, pc = PointCloud.fromJSON(data['pc']), pose = Transform.fromJSON(data['pose']))

        for node,data in output['nodes'].items():
            for target,data in data['edges'].items():
                graph.add_edge(node, target, transform =  Transform.fromJSON(data['transform']))

        return cls(graph)

    def get_nodes(self):
        for node,data in self.graph.nodes(data=True):
            yield node, data['pose'], data['pc']

    def get_edges(self):
        for edge, data in self.graph.edges.items():
            yield edge, data['transform']

    # def initialize(self, point_cloud):
    #     self.graph.

    def new_node(self, scan = None, pose=None, links = None ):
        idx = self.graph.number_of_nodes()
        self.graph.add_node(idx, pc = scan, pose = pose)

        if links != None:
            for node, relative_transform in links.items():
                self.add_edge(idx, node, relative_transform)

        return idx

    def add_edge(self, source, target, transform):
        self.graph.add_edge(source, target, transform = transform)

    def get_nearby_poses(self, location, max_distance):
        robot = location.get_components()[1]

        out = []
        for node,data in self.graph.nodes(data=True):
            loc = data['pose'].get_components()[1]
            distance = np.linalg.norm(robot - loc)
            if distance < max_distance:
                out.append( (distance, node,) )

        return sorted(out)

    def optimize(self):
        pass

    def plot(self, viz):
        for node, pose, pc in self.get_nodes():
            viz.plot_Pose(pose)

        for (x,y), transform in self.get_edges():
            p1 = self.graph.nodes[x]['pose'].get_components()[1]
            p2 = self.graph.nodes[y]['pose'].get_components()[1]
            print("ploting", p1, p2)
            viz.plot_line(p1, p2)


if __name__ == "__main__":
    a = PoseGraph()

    a.new_node()
    a.new_node()
    a.new_node()
    a.new_node()

    a.add_edge(0, 1, transform = Transform.fromComponents(0, xy = (0,1) ))
    a.add_edge(1, 2, transform = Transform.fromComponents(0, xy = (1,0) ))
    a.add_edge(2, 3, transform = Transform.fromComponents(0, xy = (0,-1) ))
    a.add_edge(3, 0, transform = Transform.fromComponents(0, xy = (-1,0) ))

    a.save("test.json")

