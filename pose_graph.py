


# NODE has:
    # ID
    # Pose
    # PC

# EDGE has:
    # ID pair
    # Transform

class PoseGraph:
    pass

    def __init__(self):
        pass
        self.nodes = []
        self.edge = []

        self.graph = nx.Graph()

        self.node_count = 0

    def initialize(self, point_cloud):
        pass
        self.graph.

    def add_node(self, node, links):
        pass

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



