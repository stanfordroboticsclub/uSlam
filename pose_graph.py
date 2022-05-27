
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
            graph.add_node(int(node), pc = PointCloud.fromJSON(data['pc']), pose = Transform.fromJSON(data['pose']))

        for node,data in output['nodes'].items():
            for target,data in data['edges'].items():
                graph.add_edge(int(node), int(target), transform =  Transform.fromJSON(data['transform']))

        return cls(graph)

    def get_nodes(self):
        for node,data in self.graph.nodes(data=True):
            if "raw_pc" in data:
                yield node, data['pose'], data['raw_pc'] # TODO raw_pc vs pc
            else:
                yield node, data['pose'], data['pc'] # TODO raw_pc vs pc

    def get_edges(self):
        for edge, data in self.graph.edges.items():
            yield edge, data['transform']

    def poses_to_pc(self):
        for node,data in self.graph.nodes(data=True):
            if self.graph.nodes[node]["pc"] != None:
                self.graph.nodes[node]["pc"].pose = data['pose']

    def new_node(self, pc = None, pose=None, links = None ):
        idx = self.graph.number_of_nodes()
        self.graph.add_node(idx, pc = pc, pose = pose)

        if links != None:
            for node, relative_transform in links.items():
                self.add_edge(idx, node, relative_transform)

        return idx

    def __repr__(self):
        out = ["PoseGraph:"]
        for node, pose, pc in self.get_nodes():
            out.append(f" Node {node}: " + pose.__repr__())

        for edge, transform in self.get_edges():
            i, j = edge
            out.append(f" Edge {i}->{j}: " + transform.__repr__())

        return "\n".join(out)


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

    def plot(self, viz, plot_pc = False):
        self.poses_to_pc()

        def center_node(node):
            print("centering node", node)

            node = int(node.strip())

            node_pose = self.graph.nodes[node]['pose']
            for target in self.graph.successors(node):
                transform = self.graph.edges[node, target]['transform']

                pose = transform.combine( node_pose ).matrix
                self.graph.nodes[target]['pose'].matrix = pose

            for source in self.graph.predecessors(node):
                transform = self.graph.edges[source, node]['transform']

                pose = transform.inv().combine( node_pose ).matrix
                self.graph.nodes[source]['pose'].matrix = pose
            self.plot(viz, plot_pc=plot_pc)

        viz.click = center_node


        colors = ["#348ABD", "#A60628", "#7A68A6", "#467821", "#CF4457", "#188487", "#E24A33" ]

        i = 0
        for node, pose, pc in self.get_nodes():
            viz.plot_Pose(pose, c=colors[i%len(colors)], tag = str(node)+"_pose")
            if plot_pc:
                global_pc = pc.global_frame()
                viz.plot_PointCloud(global_pc, c=colors[i%len(colors)], tag = str(node)+"_pc")

            i += 1

        for (x,y), transform in self.get_edges():
            p1 = self.graph.nodes[x]['pose'].get_components()[1]
            p2 = self.graph.nodes[y]['pose'].get_components()[1]

            diff = self.graph.nodes[y]['pose'].combine( self.graph.nodes[x]['pose'].inv() ).combine( transform.inv() )
            cost = np.linalg.norm( diff.matrix - np.eye(3), "fro")
            # print('edge_cost', cost)



            hex = int( min(255 * (1 / cost) , 255 ))
            # print( int(255 * (np.tanh(cost) + 1)/2 ) )
            # hex = int(255 * (np.tanh(cost) + 1)/2 )
            color = "#FF%02x%02x" %  ( hex, hex )
            viz.plot_line(p1, p2, c=color, tag = str((x,y))+"_edge")


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

