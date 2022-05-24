
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

def recover_transforms(A, hold_steady=None):
    n = A.shape[0]//3

    print(A)
    print("pos", A[ :2*n , 2*n:])
    print(n)

    u, s, vt = np.linalg.svd(A, full_matrices=False)

    A_lowrank = np.zeros((len(u), len(vt)))
    for i in range(2):
        A_lowrank += s[i] * np.outer(u.T[i], vt[i])

    transforms = []
    rank = 2
    Z = vt[:rank,:] * np.sqrt(s[:rank, None])

    print("X = Z.T @ Z", (A - Z.T @ Z) )

    Rots = []
    for i in range(n):
        Rots.append( get_rot_matirx( Z[:2, 2*i:2*i+2] ) )

    print(Rots)

    if hold_steady is None:
        global_rot = np.eye(2)
    else:
        global_rot = Rots[hold_steady].T

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

def solve_pg_paper(pg, hold_steady=0):

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
        # j, i = edge
        # angle, t_ij = data['transform'].inv().get_components()
        angle, t_ij = data['transform'].get_components()
        # t_ij = t_ij / 10
        # angle = -angle

        print(f"edge {i} -> {j}, angle={np.degrees(angle)}, t={t_ij}")
        # new = graph.nodes[j]['pose'].combine( graph.nodes[i]['pose'].inv() )
        # new_angle, new_t_ij = new.get_components()
        # print(f"calc {i} -> {j}, angle={new_angle}, t={new_t_ij}")
        print()

        R_ij = np.eye(2)
        R_ij[0,0] = np.cos(angle); R_ij[0,1] =-np.sin(angle)
        R_ij[1,0] = np.sin(angle); R_ij[1,1] = np.cos(angle)

        # print(X_Rt(i,j).shape)
        # print(X_Rt(i,i).shape)
        # print(t_ij.shape)
        # print(X_RR(i,j).shape)
        # print(R_ij.shape)

        # t_ij = - t_ij #TODO WTF WHYYY??
        cost += 10  * cp.norm( X_Rt(i,j) - X_Rt(i,i) - t_ij)
        cost += 10  * cp.norm( X_RR(i,j) - R_ij, "fro") / np.sqrt(2)

    # cost += 0.00001*cp.norm(X, "fro")
    cost += 0.00001*cp.sum(cp.abs(X))

    # constraints = [ X_Rt(0,0) == np.zeros((2))]
    # constraints = []
    constraints = [ X[2*n + hold_steady, 2*n + hold_steady] == 0 ]
    # constraints = [ X[-1,-1] == 0 ]
    for i in range(n):
        constraints.append( X_RR(i,i) == np.eye(2) )

    prob = cp.Problem(cp.Minimize(cost), constraints )
    # prob.solve(solver=cp.CVXOPT, verbose=True)
    prob.solve(solver=cp.CVXOPT, verbose=True)
    # prob.solve(verbose=True)
    # prob.solve(max_iters = 100_000)
    # prob.solve(alpha= 1.2, acceleration_lookback=0, use_indirect=False, scale=5, normalize=True)

    transforms = recover_transforms(X.value, hold_steady=hold_steady)

    for i,pose in enumerate(transforms):
        graph.nodes[i]['pose'] = Transform(pose)
        if graph.nodes[i]["pc"] != None:
            graph.nodes[i]['pc'].pose = Transform(pose)


def solve_pg_positions(pg, hold_steady=0):
    graph = pg.graph
    n = graph.number_of_nodes()

    Ts = cp.Variable( ( n, 2 ) )

    cost = 0
    for (i,j), transform in pg.get_edges():

        t_ij = transform.matrix[:2, 2]
        print(t_ij)
        R_i = pg.graph.nodes[i]['pose'].matrix[:2, :2]
        
        cost += cp.norm(  R_i.T @ ( Ts[j,:] - Ts[i,:]) - t_ij )**2

    constraints = [ Ts[hold_steady, 0] == 0 ]
    constraints += [ Ts[hold_steady, 1] == 0 ]

    prob = cp.Problem(cp.Minimize(cost), constraints )
    prob.solve(verbose=True)

    for i in range(n):
        graph.nodes[i]['pose'].matrix[:2, 2] = Ts.value[i, :2]

# np.get_printoptions()['linewidth']
np.set_printoptions(linewidth=160)
np.set_printoptions(linewidth=500)

def copy_test():
    pg = PoseGraph()

    pg.new_node()
    pg.new_node()

    # pg.add_edge(1, 0, transform = Transform.fromComponents(45, (200, 200)) )
    # pg.add_edge(0, 1, transform = Transform.fromComponents(45, (200, 200)) )

    # pg.add_edge(0, 1, transform = Transform.fromComponents(10.15257190985308, (-30.640923334094072,398.9085894000594)) )
    # pg.add_edge(0, 1, transform = Transform.fromComponents(0, (-30.640923334094072,398.9085894000594)) )

    pg.add_edge(1, 0, transform = Transform.fromComponents(-10.152571909853103, (40.154468049468605,398.0634969142168)) )



    solve_pg_paper(pg)
    print(pg)

    viz = Vizualizer(mm_per_pix=2)
    pg.plot(viz, plot_pc=False)
    viz.mainloop()

def simple_test():
    pg = PoseGraph()

    pg.new_node()
    pg.new_node()
    pg.new_node()
    pg.new_node()
    # pg.new_node()
    # pg.new_node()

    a = lambda:np.random.normal(scale=0)
    p = lambda:np.random.normal(scale=3)

    pg.add_edge(0, 1, transform = Transform.fromComponents(a(), xy = ( 0 + p(), 100 + p()) ))
    pg.add_edge(1, 2, transform = Transform.fromComponents(a(), xy = ( 100 + p(), 0 + p()) ))
    pg.add_edge(2, 3, transform = Transform.fromComponents(a(), xy = ( 0 + p(),-100 + p()) ))
    # pg.add_edge(3, 0, transform = Transform.fromComponents(a(), xy = (-100 + p(), 0 + p()) ))

    # pg.add_edge(1, 4, transform = Transform.fromComponents(90 + a(), xy = (0 + p(), 100 + p()) ))
    # pg.add_edge(2, 5, transform = Transform.fromComponents(90 + a(), xy = (0 + p(), 100 + p()) ))
    # pg.add_edge(4, 5, transform = Transform.fromComponents(a(), xy = (0 + p(),-100 + p()) ))


    solve_pg_paper(pg)
    # solve_pg_positions(pg)


    # pg = PoseGraph.load("test.json")
    pg.save("test.json")
    viz = Vizualizer(mm_per_pix= 2 )
    pg.plot(viz, plot_pc=False)
    viz.mainloop()

def load():
    viz = Vizualizer(mm_per_pix=15)
    pg = PoseGraph.load("t.json")
    # pg = PoseGraph.load("data/villan/hamilton_2.graph.json")


    # for node, pose, pc in pg.get_nodes():
    #     if pc != None:
    #         pc.scale(0.1)

        # if pose != None:
        #     pose.scale(0.1)

    # for i in range(pg.graph.number_of_nodes()):
    #     if i not in [0,1, 2, 3]:
    #         pg.graph.remove_node(i)

    nodes_to_edges(pg)

    print(pg)
    # solve_pg_paper(pg)
    solve_pg_positions(pg)
    print(pg)

    pg.plot(viz, plot_pc=True)
    viz.mainloop()

def nodes_to_edges(pg):
    "fox for badly daved graphs"

    for (x,y), transform in pg.get_edges():
        t1 = pg.graph.nodes[x]['pose']
        t2 = pg.graph.nodes[y]['pose']

        pg.graph.edges[x,y]['transform'] = t2.combine( t1.inv() )



if __name__ == "__main__":
    # simple_test()
    # copy_test()
    load()










