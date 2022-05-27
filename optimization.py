
from utils import Transform
import networkx as nx
import cvxpy as cp
import numpy as np

from pose_graph import PoseGraph
from output import Vizualizer

from time import sleep


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

        cost += 10  * cp.norm( X_Rt(i,j) - X_Rt(i,i) - t_ij)
        cost += 10  * cp.norm( X_RR(i,j) - R_ij, "fro") / np.sqrt(2)

    # cost += 0.00001*cp.norm(X, "fro")
    # cost += 0.00001*cp.sum(cp.abs(X))

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
        
        # cost += cp.norm(  R_i.T @ ( Ts[j,:] - Ts[i,:]) - t_ij )**2
        # cost += cp.sum_squares(  R_i.T @ ( Ts[j,:] - Ts[i,:]) - t_ij ) # from paper and logic

        cost += cp.sum_squares(    - R_j @ R_i.T @ Ts[i,:] + Ts[j,:] - t_ij  )
        # cost += cp.norm(    - R_j @ R_i.T @ Ts[i,:] + Ts[j,:] - t_ij  )

        part_cost = np.linalg.norm(  R_i.T @ ( real_tj - real_ti) - t_ij )**2
        
        part_cost = np.linalg.norm(  - R_j @ R_i.T @ real_ti + real_tj - t_ij )**2
        # assert part_cost < 1e-4
        real_cost += part_cost

        # [ Rj tj    [ Ri.T -Ri.T @ ti,
        #   0   1] @ [ 0       1 ] 

        # [ Rj @ Ri.T,  - Rj @ Ri.T @ ti + tj
        #   0               1   ]

        print(f"{i=}; {j=}; {part_cost=}; {real_ti=}; {real_tj=}; {t_ij=}; {R_i=} ")

        # t1 = pg.graph.nodes[i]['pose']
        # t2 = pg.graph.nodes[j]['pose']

        # assert np.allclose(relative , t2.combine( t1.inv() ).matrix)
        # assert np.allclose(relative , pose_j @ np.linalg.inv( pose_i) )

    print(f"real_cost = {real_cost}")

    constraints = [ Ts[hold_steady, 0] == 0 ]
    constraints += [ Ts[hold_steady, 1] == 0 ]
    # constraints = []

    prob = cp.Problem(cp.Minimize(cost), constraints )
    prob.solve(verbose=True)

    for i in range(n):
        graph.nodes[i]['pose'].matrix[:2, 2] = Ts.value[i, :2]


def solve_pg_rotations(pg, hold_steady = 0):

    graph = pg.graph
    n = graph.number_of_nodes()

    # for node in range(n):
    #     if len(list(nx.all_neighbors(graph, node))) > 1:
    #         queue = [node]

    queue = [np.random.randint(n)]

    if queue == []:
        print("emplty")
        return

    # for _ in range(10 * n):
    for _ in range(1):
        print(queue)
        if queue == []:
            break
        
        node = queue.pop(0)

        # node = np.random.randint(n) # TODO smarted sampling

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
        # graph.nodes[node]['pose'].matrix[:2, 2] = sum(pos)/len(pos)



# np.get_printoptions()['linewidth']
np.set_printoptions(linewidth=160)
np.set_printoptions(linewidth=500)

def copy_test():
    pg = PoseGraph()

    pg.new_node(pose=Transform.Identity())
    pg.new_node(pose=Transform.fromComponents(45, (100,40)))
    pg.new_node(pose=Transform.fromComponents(45, (100,40)))
    pg.new_node(pose=Transform.fromComponents(45, (100,40)))

    pg.add_edge(0, 1, transform = Transform.Identity() )
    pg.add_edge(1, 2, transform = Transform.Identity() )
    pg.add_edge(2, 3, transform = Transform.Identity() )

    # pg.add_edge(1, 0, transform = Transform.fromComponents(45, (200, 200)) )
    # pg.add_edge(1, 0, transform = Transform.fromComponents(45, (200, 200)) )
    # pg.add_edge(0, 1, transform = Transform.fromComponents(45, (200, 200)) )

    # pg.add_edge(0, 1, transform = Transform.fromComponents(10.15257190985308, (-30.640923334094072,398.9085894000594)) )
    # pg.add_edge(0, 1, transform = Transform.fromComponents(0, (-30.640923334094072,398.9085894000594)) )

    # pg.add_edge(0, 1, transform = Transform.fromComponents(-45, (40.154468049468605,398.0634969142168)) )

    return pg

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


    real_transform = Transform.fromComponents(45, xy = (0, 1000) )
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
    pg = PoseGraph.load("output_sim.json")

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
    pg, mm_per_pix, plot_pc = simple_test()
    # pg, mm_per_pix, plot_pc = load()
    print(pg)

    viz = Vizualizer(mm_per_pix=mm_per_pix)

    # viz.update()

    # solve_pg_paper(pg)

    pg.plot(viz, plot_pc=plot_pc)

    # for _ in range(10):
    def opt():
        # solve_pg_positions(pg)
        solve_pg_rotations(pg)
        print(pg)
        # viz.clear()
        pg.plot(viz, plot_pc=plot_pc)
        viz.after(100, opt)


    # viz.after(100, opt)

    viz.mainloop()

if __name__ == "__main__":
    main()










