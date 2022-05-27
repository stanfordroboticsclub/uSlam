
from utils import Transform
import networkx as nx
import cvxpy as cp
import numpy as np

from pose_graph import PoseGraph
from output import Vizualizer

from time import sleep

import matplotlib.pyplot as plt


def graph_loss(pg):
    loss = 0

    for (i,j), transform in pg.get_edges():

        relative = pg.graph.edges[i,j]['transform'].matrix
        pose_i = pg.graph.nodes[i]['pose'].matrix
        pose_j = pg.graph.nodes[j]['pose'].matrix

        t_ij = transform.matrix[:2, 2]
        R_ij = transform.matrix[:2, :2]

        R_i = pg.graph.nodes[i]['pose'].matrix[:2, :2]
        R_j = pg.graph.nodes[j]['pose'].matrix[:2, :2]

        real_ti = pg.graph.nodes[i]['pose'].matrix[:2,2]
        real_tj = pg.graph.nodes[j]['pose'].matrix[:2,2]

        # part_cost = np.linalg.norm(  R_i.T @ (real_tj - real_ti) - t_ij )**2
        part_cost = np.linalg.norm(  - R_j @ R_i.T @ real_ti + real_tj - t_ij )**2

        part_cost += np.linalg.norm(  R_i.T @ R_j - R_ij, "fro")**2

        loss += part_cost
    return loss


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

    scale_down = 1000

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
        angle, t_ij = data['transform'].get_components()
        print(f"edge {i} -> {j}, angle={np.degrees(angle)}, t={t_ij}")

        R_ij = data['transform'].matrix[:2, :2]
        t_ij = data['transform'].matrix[:2, 2]

        t_ij = t_ij / scale_down

        # R_ij = np.eye(2)
        # R_ij[0,0] = np.cos(angle); R_ij[0,1] =-np.sin(angle)
        # R_ij[1,0] = np.sin(angle); R_ij[1,1] = np.cos(angle)

        cost += 10  * cp.sum_squares( X_Rt(i,j) - X_Rt(i,i) - t_ij)
        cost += 10  * cp.sum_squares( X_RR(i,j) - R_ij) / np.sqrt(2)
        # cost += 10  * cp.norm( X_RR(i,j) - R_ij, "fro") / np.sqrt(2)

    # cost += 0.0001*cp.norm(X, "fro")
    # cost += 0.00001*cp.sum(cp.abs(X))

    constraints = []
    constraints = [ X[2*n + hold_steady, 2*n + hold_steady] == 0 ]
    for i in range(n):
        constraints.append( X_RR(i,i) == np.eye(2) )

    prob = cp.Problem(cp.Minimize(cost), constraints )

    prob.solve(solver=cp.CVXOPT, verbose=True)
    # prob.solve(verbose=True)
    # prob.solve(alpha= 1.2, acceleration_lookback=0, use_indirect=False, scale=5, normalize=True)

    transforms = recover_transforms(X.value, hold_steady=hold_steady)

    for i,pose in enumerate(transforms):
        graph.nodes[i]['pose'] = Transform(pose)
        graph.nodes[i]['pose'].matrix[:2, 2] *= scale_down

        if graph.nodes[i]["pc"] != None:
            graph.nodes[i]['pc'].pose = Transform(pose)


def log_matrix(M):
    assert M.shape == (2,2)
    return np.arctan2(M[1,0], M[0,0] )

def exp_matrix(angle):
    matrix = np.eye(2)
    matrix[0,0] = np.cos(angle); matrix[0,1] =-np.sin(angle)
    matrix[1,0] = np.sin(angle); matrix[1,1] = np.cos(angle)
    return matrix

def avg_rotations(rotations):
    R = rotations[0]
    n = len(rotations)
    while 1:
        r = sum( log_matrix( R.T @ r) for r in rotations) / n
        if r < 1e-5:
            return R
        R = R @ exp_matrix(r)


def solve_pg_positions(pg, hold_steady=0):
    graph = pg.graph
    n = graph.number_of_nodes()

    Ts = cp.Variable( ( n, 2 ) )

    cost = 0
    real_cost = 0
    for (i,j), transform in pg.get_edges():

        relative = pg.graph.edges[i,j]['transform'].matrix
        pose_i = pg.graph.nodes[i]['pose'].matrix
        pose_j = pg.graph.nodes[j]['pose'].matrix

        t_ij = transform.matrix[:2, 2]
        R_i = pg.graph.nodes[i]['pose'].matrix[:2, :2]
        R_j = pg.graph.nodes[j]['pose'].matrix[:2, :2]

        real_ti = pg.graph.nodes[i]['pose'].matrix[:2,2]
        real_tj = pg.graph.nodes[j]['pose'].matrix[:2,2]
        
        # cost += cp.sum_squares(  R_i.T @ ( Ts[j,:] - Ts[i,:]) - t_ij ) # from paper and logic
        cost += cp.sum_squares(    - R_j @ R_i.T @ Ts[i,:] + Ts[j,:] - t_ij  )

    constraints = [ Ts[hold_steady, 0] == np.array([0,0]) ]
    # constraints = []

    prob = cp.Problem(cp.Minimize(cost), constraints )
    prob.solve()

    for i in range(n):
        graph.nodes[i]['pose'].matrix[:2, 2] = Ts.value[i, :2]


def solve_pg_rotations(pg, hold_steady = 0, also_positions = False):

    graph = pg.graph
    n = graph.number_of_nodes()

    for node in range(n):
        if len(list(nx.all_neighbors(graph, node))) > 1:
            queue = [node]

    # queue = [np.random.randint(n)]

    if queue == []:
        print("emplty")
        return

    for _ in range(10 * n):
        if queue == []:
            break

        node = queue.pop(0)

        if node == hold_steady:
            continue
        rots = []
        pos = []

        for target in graph.successors(node):
            transform = graph.edges[node, target]['transform']
            target_pose = graph.nodes[target]['pose']

            pose = transform.inv().combine( target_pose ).matrix
            rots.append(pose[:2, :2])
            pos.append(pose[:2, 2] )

            queue.append(target)

        for source in graph.predecessors(node):
            transform = graph.edges[source, node]['transform']
            source_pose = graph.nodes[source]['pose']

            pose = transform.combine( source_pose ).matrix
            rots.append(pose[:2, :2])
            pos.append(pose[:2, 2] )

            queue.append(source)

        if rots == []:
            continue
        graph.nodes[node]['pose'].matrix[:2, :2] = avg_rotations(rots)
        if also_positions:
            graph.nodes[node]['pose'].matrix[:2, 2] = sum(pos)/len(pos)



np.set_printoptions(linewidth=160)
np.set_printoptions(linewidth=500)


def simple_test():
    pg = PoseGraph()

    # pg.new_node( pose = Transform.fromComponents(0, xy = ( 0, 0) ) )
    # pg.new_node( pose = Transform.fromComponents(0, xy = ( 1000, 0) ) )
    # pg.new_node( pose = Transform.fromComponents(-45, xy = ( 1000, 1000) ) )
    # pg.new_node( pose = Transform.fromComponents(143, xy = ( 0, 1000) ) )


    # pg.add_edge(0, 1, transform = Transform.fromComponents(0, xy = (1000,0) ))
    # pg.add_edge(1, 2, transform = Transform.fromComponents(0, xy = (0, 1000) ))
    # pg.add_edge(2, 3, transform = Transform.fromComponents(0, xy = ( -1000, 0) ))
    # pg.add_edge(3, 0, transform = Transform.fromComponents(0, xy = (0, -1000) ))

    # pg.new_node( pose = Transform.fromComponents(0, xy = ( 0, 0) ) )
    # pg.new_node( pose = Transform.fromComponents(95, xy = ( 0, 1100) ) )
    # pg.new_node( pose = Transform.fromComponents(190, xy = ( -1100, 1100) ) )
    # pg.new_node( pose = Transform.fromComponents(285, xy = ( -1200,    0) ) )

    # pg.add_edge(0, 1, transform = Transform.fromComponents(90, xy = (0, 1000) ))
    # pg.add_edge(1, 2, transform = Transform.fromComponents(90, xy = (0, 1000) ))
    # pg.add_edge(2, 3, transform = Transform.fromComponents(90, xy = (0, 1000) ))
    # pg.add_edge(3, 0, transform = Transform.fromComponents(90, xy = (0, 1000) ))


    real_transform = Transform.fromComponents(44, xy = (0, 1010) )
    fake_transform = Transform.fromComponents(35, xy = (0, 1100) )

    current = Transform.Identity()
    for i in range(8):
        pg.new_node( pose = current.copy() )
        current = fake_transform.combine( current )

    for i in range(8):
        pg.add_edge(i, (i+1)%8, transform = real_transform.copy() )

    return pg, 15, False

def load():
    # pg = PoseGraph.load("t.json")
    # pg = PoseGraph.load("output_looop_real.json")
    # pg = PoseGraph.load("output_messy_perf.json")
    # pg = PoseGraph.load("output_first_perf.json")
    # pg = PoseGraph.load("output_sim.json")
    pg = PoseGraph.load("output_simple_working.json")
    # pg = PoseGraph.load("output.json")

    # for i in range(pg.graph.number_of_nodes()):
    #     if i not in [0,1, 2, 3]:
    #         pg.graph.remove_node(i)

    # nodes_to_edges(pg)
    return pg, 15, True


def nodes_to_edges(pg):
    "fox for badly saved graphs"

    for (x,y), transform in pg.get_edges():
        t1 = pg.graph.nodes[x]['pose']
        t2 = pg.graph.nodes[y]['pose']

        pg.graph.edges[x,y]['transform'] = t2.combine( t1.inv() )
        assert np.allclose( pg.graph.edges[x,y]['transform'].matrix, t2.combine( t1.inv() ).matrix)



def main():
    # pg, mm_per_pix, plot_pc = simple_test()
    pg, mm_per_pix, plot_pc = load()
    print(pg)

    viz = Vizualizer(mm_per_pix=mm_per_pix)


    loss = []
    # loss = solve_pg_paper(pg)

    pg.plot(viz, plot_pc=plot_pc)


    def opt():
        # loss.append(graph_loss(pg))
        # solve_pg_positions(pg)
        loss.append(graph_loss(pg))
        solve_pg_rotations(pg, also_positions=True)


        pg.plot(viz, plot_pc=plot_pc)
        viz.after(100, opt)

    viz.after(5000, opt)


    def quit():
        viz.destroy()
        plt.figure()
        plt.plot(loss)
        plt.show()

    viz.protocol("WM_DELETE_WINDOW", quit)

    viz.mainloop()

if __name__ == "__main__":
    main()










