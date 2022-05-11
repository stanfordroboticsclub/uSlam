
from utils import Transform
import networkx as nx
import cvxpy as cp
import numpy as np




graph = nx.Graph()


graph.add_node(0)
graph.add_node(1)
graph.add_node(2)
graph.add_node(3)


graph.add_edge(0, 1, measure = Transform.fromComponents(0, xy = (0,1) ))
graph.add_edge(1, 2, measure = Transform.fromComponents(0, xy = (1,0) ))
graph.add_edge(2, 3, measure = Transform.fromComponents(0, xy = (0,-1) ))
graph.add_edge(3, 0, measure = Transform.fromComponents(0, xy = (-1.5,0) ))





def solve_pose_graph(graph):

    n = graph.number_of_nodes()

    X = cp.Variable( ( 3*n, 3*n ) )

    Z = cp.Variable( ( 2, 3*n ) )


    def X_RR(i,j):
        return X[2*i:2*i+2, 2*j:2*j+2]

    def X_Rt(i,j):
        return X[2*i : 2*i+2 , 2*n + j]

    cost = 0
    for edge, data in graph.edges.items():
        i, j = edge
        angle, t_ij = data['measure'].get_components()

        R_ij = np.eye(2)
        R_ij[0,0] = np.cos(angle); R_ij[0,1] =-np.sin(angle)
        R_ij[1,0] = np.sin(angle); R_ij[1,1] = np.cos(angle)

        # print(X_Rt(i,j).shape)
        # print(X_Rt(i,i).shape)
        # print(t_ij.shape)
        # print(X_RR(i,j).shape)
        # print(R_ij.shape)
        cost += 10  * cp.norm( X_Rt(i,j) - X_Rt(i,i) - t_ij)
        cost += 0.1 * cp.norm( X_RR(i,j) - R_ij, "fro") / np.sqrt(2)

    constraints = [  X == Z.T @ Z ]
    for i in range(n):
        constraints.append( X_RR(i,i) == np.eye(2) )


    prob = cp.Problem(cp.Minimize(cost), constraints )
    prob.solve()

solve_pose_graph(graph)







