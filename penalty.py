#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 16 10:33:03 2021

@author: algutier
"""

import igraph as ig
import math as m

from grid_generation import *
from instance import *


def floyd_warshall_full(graph):
    """
    Version N3 in space
    """
    n = len(graph.vs)
    d = [[[m.inf] * n for i in range(n)] for i in range(n)]
    d[0], nxt = init(graph)
    for k in range(1, n):
        for i in range(n):
            for j in range(n):
                d[k][i][j] = min(d[k - 1][i][j], d[k-1][i][k] + d[k - 1][k][j])
    return d[n - 1]

def floyd_warshall(graph):
    """
    The floyd-warshall algorithm to find all the distances between a pair 

    Parameters
    ----------
    graph : Graph
        The graph to inspect.

    Returns
    -------
    d : List<List<int>>
        The matrix of the distance between all pairs of nodes in the graph.
    nxt : List<List<int>>
        The matrix which contains in (i, j) the index of the next node to 
        follow for the shortest path from i to j.

    """
    n = len(graph.vs)
    d, nxt = init(graph)
    for k in range(1, n):
        for i in range(n):
            for j in range(n):
                by_k = d[i][k] + d[k][j]
                if by_k < d[i][j]:
                    d[i][j] = by_k
                    nxt[i][j] = nxt[i][k]
    return d, nxt

def init(graph):
    """
    Initialization for the floyd-warshall algorithm : adjacency matrix with 
    infinity and next for the shortest path.

    Parameters
    ----------
    graph : Graph
        The graph to inspect.

    Returns
    -------
    adj : List<List<int>>
        The adjacency matrix of the graph, infinity if there are no connexions
        between two nodes.
    nxt : List<List<int>>
        The initialization of the matrix of the next node to follow to find 
        the shortest path.

    """
    n = len(graph.vs)
    adj = [[m.inf] * n for i in range(n)]
    nxt = [[None] * n for i in range(n)]
    for edge in graph.es:
        u, v = edge.tuple
        adj[u][v] = 1
        adj[v][u] = 1
        nxt[u][v] = v
        nxt[v][u] = u
    for i in range(n):
        adj[i][i] = 0
        nxt[i][i] = i
    return adj, nxt

if __name__ == "__main__":
    g = ig.Graph(7)
    g.add_edges([(0, 1), (1, 2), (1, 3), (2, 6), (2, 3), (2, 5), (3, 4),
                 (4, 5), (5, 6)])
    
    for f in g.es:
        print(f.tuple)
    
    g = Grid(3, 3).graphe_m
    i = Instance(g, g, [], [], "")
    g = i.deterministic_graph
    d1 = floyd_warshall_full(g)
    print()
    for l in d1:
        print(l)
    print()
    d2, n = floyd_warshall(g)
    print()
    
    for l in d2:
        print(l)
    print()
    for l in n:
        print(l)
    print()
        
    i.print_grid()

        
    for k in range(len(d1)):
        for x, y in zip(d1[k],d2[k]):
            assert(x == y)
    print()
    print(g)