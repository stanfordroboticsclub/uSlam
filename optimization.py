
from utils import Transform
import networkx as nx
import cvxpy as cp
import numpy as np

from pose_graph import PoseGraph
from output import Vizualizer


# graph = nx.Graph()

# graph.add_node(0)
# graph.add_node(1)
# graph.add_node(2)
# graph.add_node(3)


# graph.add_edge(0, 1, measure = Transform.fromComponents(0, xy = (0,1) ))
# graph.add_edge(1, 2, measure = Transform.fromComponents(0, xy = (1,0) ))
# graph.add_edge(2, 3, measure = Transform.fromComponents(0, xy = (0,-1) ))
# graph.add_edge(3, 0, measure = Transform.fromComponents(0, xy = (-1,0) ))


def get_rot_matirx(A):
    u, s, vt = np.linalg.svd(A, full_matrices=False)
    return u @ vt

def rank2_approx(A):
    n = A.shape[0]//3

    print(A)
    print("pos", A[ :2*n , 2*n:])
    print(n)

    u, s, vt = np.linalg.svd(A, full_matrices=False)

    A_lowrank = np.zeros((len(u), len(vt)))
    for i in range(2):
        A_lowrank += s[i] * np.outer(u.T[i], vt[i])

    # print(u.T)
    # print()
    # print(vt)
    # print()
    print(s)
    # print()

    # print(u.shape)
    # print(s.shape)
    # print(vt.shape)

    # print("remake ", np.isclose(A, u * s @ vt) )

    transforms = []
    rank = 2
    Z = vt[:rank,:] * np.sqrt(s[:rank, None])

    print("X = Z.T @ Z", (A - Z.T @ Z) )
    # print(Z.shape)

    Rots = []
    for i in range(n):
        Rots.append( get_rot_matirx( Z[:2, 2*i:2*i+2] ) )

    print(Rots)

    global_rot = Rots[-1].T

    # print(np.hstack(Rots).shape)
    # print(A[ :2*n , 2*n:].shape)
    ts = 1/n * np.hstack(Rots) @ A[ :2*n , 2*n:]

    transforms = []
    for i in range(n):
        Q = np.eye(3)
        Q[:2, :2] = global_rot @ Rots[i]
        Q[:2, -1] = global_rot @ ts[:, i]
        transforms.append(Q)
        print(Q)

    return transforms

def solve_pose_graph(pg, hold_steady=None):

    graph = pg.graph

    n = graph.number_of_nodes()

    X = cp.Variable( ( 3*n, 3*n ) , PSD = True)

    def X_RR(i,j):
        return X[2*i:2*i+2, 2*j:2*j+2]

    def X_Rt(i,j):
        return X[2*i : 2*i+2 , 2*n + j]

    cost = 0
    for edge, data in graph.edges.items():
        i, j = edge
        angle, t_ij = data['transform'].get_components()

        R_ij = np.eye(2)
        R_ij[0,0] = np.cos(angle); R_ij[0,1] =-np.sin(angle)
        R_ij[1,0] = np.sin(angle); R_ij[1,1] = np.cos(angle)

        # print(X_Rt(i,j).shape)
        # print(X_Rt(i,i).shape)
        # print(t_ij.shape)
        # print(X_RR(i,j).shape)
        # print(R_ij.shape)

        cost += 10  * cp.norm( X_Rt(i,j) - X_Rt(i,i) - t_ij)
        cost += 10  * cp.norm( X_RR(i,j) - R_ij, "fro") / np.sqrt(2)

    # constraints = [ X_Rt(0,0) == np.zeros((2))]
    # constraints = []
    constraints = [ X[-1,-1] == 0 ]
    for i in range(n):
        constraints.append( X_RR(i,i) == np.eye(2) )

    prob = cp.Problem(cp.Minimize(cost), constraints )
    prob.solve()

    transforms = rank2_approx(X.value)

    for i,pose in enumerate(transforms):
        graph.nodes[i]['pose'] = Transform(pose)



# np.get_printoptions()['linewidth']
np.set_printoptions(linewidth=160)
np.set_printoptions(linewidth=500)

pg = PoseGraph()

pg.new_node()
pg.new_node()
pg.new_node()
pg.new_node()

pg.add_edge(0, 1, transform = Transform.fromComponents(0, xy = (0,1) ))
pg.add_edge(1, 2, transform = Transform.fromComponents(0, xy = (1,0) ))
pg.add_edge(2, 3, transform = Transform.fromComponents(0, xy = (0,-1) ))
pg.add_edge(3, 0, transform = Transform.fromComponents(0, xy = (-1,0) ))


solve_pose_graph(pg)

# pg = PoseGraph.load("test.json")
pg.save("test.json")

viz = Vizualizer(mm_per_pix= 1/300 )

pg.plot(viz)

viz.mainloop()








