
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
            output["nodes"][node] = {"pose": pose.toJSON() , "pc": pc.toJSON() , "edges": {}}

        output["edges"] = {}
        for (x,y), transform in self.edges():
            output["nodes"][x]["edges"][y] = {"tranform": transform.toJSON() }

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
                graph.add_edge(node, target, transform =  Transform.fromJSON(data['tranform']))

        return cls(graph)

    def get_nodes(self):
        for node,data in self.graph.nodes(data=True):
            yield node, data['pose'], data['pc']

    def get_edge(self.):
        for edge, data in self.graph.edges.items():
            yield edge, data['tranform']

    # def initialize(self, point_cloud):
    #     self.graph.

    def new_node(self, node, scan = None, pose=None, links = None ):
        idx = self.graph.number_of_nodes()
        self.graph.add_node(idx, pc = scan, pose = pose)

        if links != None:
            for node, relative_transform in links.items():
                self.graph.add_edge(idx, node, transform = relative_transform)

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



