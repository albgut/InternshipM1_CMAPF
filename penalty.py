#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 16 10:33:03 2021

@author: algutier
"""

import igraph as ig
import math as m
from copy import deepcopy
from multi_a_star import build_path
import time as t

from grid_generation import *
from instance import *


def floyd_warshall_full(graph):
    """
    Version N3 in space
    """
    n = len(graph.vs)
    d = [[[m.inf] * n for i in range(n)] for i in range(n)]
    #d[0], nxt, d_without = init(graph)
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
    #d, nxt, d_without = init(graph)
    d, nxt = init(graph)
    for k in range(1, n):
        for i in range(n):
            for j in range(n):
                #update_dico(d_without, k, i, j)
                by_k = d[i][k] + d[k][j]
                if by_k < d[i][j]:
                    d[i][j] = by_k
                    nxt[i][j] = nxt[i][k]
    d_without = dict()
    for e in graph.es:
        if e["proba"] != 0:
            u, v = e.tuple
            if u > v:
                tmp = u
                u = v
                v = tmp
            d_without_edge(d, (u, v), graph, nxt, d_without)
    print("FLOYD WARSHALL COMPLETE")
            
    return d, nxt, d_without

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
    #d_without = dict()
    for edge in graph.es:
        u, v = edge.tuple
        """
        for key in d_without.keys():
            d_without[key][u][v] = 1
            d_without[key][v][u] = 1
        if edge["proba"] != 0:
            if u > v:
                tmp = u
                u = v
                v = tmp
            d_without[(u, v)] = deepcopy(adj)
            d_without[(u, v)][u][v] = m.inf
            d_without[(u, v)][v][u] = m.inf
        """
        adj[u][v] = 1
        adj[v][u] = 1
        nxt[u][v] = v
        nxt[v][u] = u
    for i in range(n):
        """
        for key in d_without.keys():
            d_without[key][i][i] = 0
        """
        adj[i][i] = 0
        
    print("INIT COMPLETE")
        
    """
    print("ADJ")
    print()
    for r in adj:
        print(r)
    print()
    
    print("nxt")
    print()
    for r in nxt:
        print(r)
    print()
    
    for edge in d_without.keys():
        print("FOR EDGE ", edge)
        print()
        for t in d_without[edge]:
            print(t)
        print()
    print()
    """
    return adj, nxt#, d_without

def floyd_warshall_bis(graph):
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
    d, nxt, d_without = init_bis(graph)
    #d, nxt = init(graph)
    for k in range(1, n):
        for i in range(n):
            for j in range(n):
                update_dico(d_without, k, i, j)
                by_k = d[i][k] + d[k][j]
                if by_k < d[i][j]:
                    d[i][j] = by_k
                    nxt[i][j] = nxt[i][k]
    print("FLOYD WARSHALL COMPLETE")
    return d, nxt, d_without

def init_bis(graph):
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
    d_without = dict()
    for edge in graph.es:
        u, v = edge.tuple
        
        for key in d_without.keys():
            d_without[key][u][v] = 1
            d_without[key][v][u] = 1
        if edge["proba"] != 0:
            if u > v:
                tmp = u
                u = v
                v = tmp
            d_without[(u, v)] = deepcopy(adj)
            d_without[(u, v)][u][v] = m.inf
            d_without[(u, v)][v][u] = m.inf
        
        adj[u][v] = 1
        adj[v][u] = 1
        nxt[u][v] = v
        nxt[v][u] = u
    for i in range(n):
        
        for key in d_without.keys():
            d_without[key][i][i] = 0
        
        adj[i][i] = 0
        
    print("INIT COMPLETE")
        
    """
    print("ADJ")
    print()
    for r in adj:
        print(r)
    print()
    
    print("nxt")
    print()
    for r in nxt:
        print(r)
    print()
    
    for edge in d_without.keys():
        print("FOR EDGE ", edge)
        print()
        for t in d_without[edge]:
            print(t)
        print()
    print()
    """
    return adj, nxt, d_without

def update_dico(d_without, k, i, j):
    """
    TODO
    """
    for d in d_without.values():
        by_k = d[i][k] + d[k][j]
        if by_k < d[i][j]:
            d[i][j] = by_k

"""
t / edge
b,a tq next(a,t)=b
faire en dynamique pour les g_t en utilisant directement nxt.
"""
def a_star_pen(graphe, start_node, end_node, d_without, d):
    """
    TODO
    
    FAIRE UN FOLLOW PATH A PARTIR DE NEXT ON S ARRETE ET ON ETIQUETTTE LES
    NODES RENCONTRES 
    
    POUR ARRET ON A * t DEPILE
                    * UNE VALEUR PLUS GRANDE (DEJA ETIQUETER OUI)
                        SI ETTIQUETEE OUI 
        
        NON ON FAIT POUR TOUS LES T !
                        
    m.inf si pas de sol
                        
    u etiquete non
    t a oui
    tt le reste a inconnu.
    
    - suit pcc par les next, si rencontre u non (pcc passe par u,v) => tous non
    - si oui t (sans u) => tous oui
    """
    open_heap = []
    closed_set = set()
    previous_node = dict()
    #((f,h,g),node)
    h_cost = d[start_node][end_node]
    heappush(open_heap, ((h_cost, h_cost, 0), start_node))
    previous_node[start_node] = None
    while not open_heap == []:
        (f, h, g), node = heappop(open_heap)
        closed_set.add(node)
        if node == end_node:
            return build_path(previous_node, end_node)
        ens_neighbors = graphe.neighbors(node)
        for neighbor in ens_neighbors:
            if not neighbor in closed_set:
                edge = graphe.get_eid(node, neighbor)
                p = graphe.es[edge]["proba"]
                dist_curr_to_neigh = penalty(neighbor, node, end_node, 
                                             p, d, d_without)
                new_g_cost = g + dist_curr_to_neigh
                new_h_cost = d[neighbor][end_node]
                new_f_cost = new_g_cost + new_h_cost
                open_neighbor = find_in_open(open_heap, neighbor)
                if not open_neighbor == None:
                    (old_f, old_h, old_g), _ = open_neighbor
                    if new_g_cost < old_g:
                        open_neighbor = ((new_g_cost + new_h_cost,
                                          new_h_cost, new_g_cost), neighbor)
                        previous_node[neighbor] = node
                else:
                    open_neighbor = ((new_g_cost + new_h_cost,
                                      new_h_cost, new_g_cost), neighbor)
                    previous_node[neighbor] = node
                heappush(open_heap, open_neighbor)
    return None
    
def penalty(neighbor, node, end_node, p, d, d_without):
    """
    TODO
    """
    if p == 0:
        return d[node][neighbor]
    if neighbor < node:
        dist_sans_u_v = d_without[(neighbor, node)][node][end_node]
    else:
        dist_sans_u_v = d_without[(node, neighbor)][node][end_node]
    dist_avec_u_v = d[neighbor][end_node] + d[node][neighbor]
    return abs(dist_sans_u_v - dist_avec_u_v) * p + d[node][neighbor]


def create_g_t(graph, nxt, t):
    """
    TODO
    """
    g_t = ig.Graph(len(graph.vs), directed=True)
    for i in range(len(nxt)):
        b = nxt[i][t]
        if b != None:
            g_t.add_edge(b, i)
    return g_t

def a_star_without_edge(good_values, start_node, end_node, graphe, edge, d_w_current):
    open_heap = []
    closed_set = set()
    i, j = edge
    #((f,h,g),node)
    h_cost = euclid(graphe, start_node, end_node)
    heappush(open_heap, ((h_cost, h_cost, 0), start_node))
    while not open_heap == []:
        #print(open_heap)
        (f, h, g), node = heappop(open_heap)
        closed_set.add(node)
        if good_values[start_node][node] != True:
            d_w_current[start_node][node] = g
            good_values[start_node][node] = True
            d_w_current[node][start_node] = g
            good_values[node][start_node] = True
        else:
            """
            print("node ", start_node, " to ", node)
            print("except ", i, ', ', j)
            print(good_values[start_node][node])
            print(d_w_current[start_node][node])
            print(g)
            """
            assert(d_w_current[start_node][node] == g)
        if node == end_node:
            #print("end astar")
            return True
        ens_neighbors = graphe.neighbors(node)
        if node == i:
            ens_neighbors.remove(j)
        if node == j:
            ens_neighbors.remove(i)
        for neighbor in ens_neighbors:
            if not neighbor in closed_set:
                new_g_cost = g + 1
                """
                if good_values[neighbor][end_node] == True:
                    new_h_cost = d_w_current[neighbor][end_node]
                else:
                    new_h_cost = euclid(graphe, neighbor, end_node)
                """
                new_h_cost = euclid(graphe, neighbor, end_node)
                new_f_cost = new_g_cost + new_h_cost
                open_neighbor = find_in_open(open_heap, neighbor)
                if not open_neighbor == None:
                    (old_f, old_h, old_g), _ = open_neighbor
                    if new_g_cost < old_g:
                        open_neighbor = ((new_g_cost + new_h_cost,
                                          new_h_cost, new_g_cost), neighbor)
                else:
                    open_neighbor = ((new_g_cost + new_h_cost,
                                      new_h_cost, new_g_cost), neighbor)
                heappush(open_heap, open_neighbor)
    return None

def euclid(graph, i, j):
    """
    TODO
    problem heuristic pas valide ....
    """
    x1 = graph.vs[i]["x_coord"]
    y1 = graph.vs[i]["y_coord"]
    x2 = graph.vs[j]["x_coord"]
    y2 = graph.vs[j]["y_coord"]
    
    return m.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    
    

def d_without_edge(d, edge, graph, nxt, d_without):
    """
    TODO
    edge donné sous la forme (u, v) avec u < v 
    """
    (u, v) = edge
    d_w_current = deepcopy(d)
    good_values = [[None] * len(d) for i in range(len(d))]
    for i in range(len(d)):
        for j in range(len(d)):
            if i == j:
                good_values[i][j] = True
            if nxt[i][j] == u or nxt[i][j] == v:
                good_values[i][j] = False
    """
    change = True
    nbchange = 0
    nbastar = 0
    """
    for i in range(len(d)):
        for j in range(len(d)):
            """
            if change:
                print()
                nbchange += 1
                print("nb of changes = ", nbchange)
                print("nb astar = ", nbastar)
                for c in good_values:
                    print(c)
                print()
                change = False
            """
            if good_values[i][j] == None:
                k = i
                l = j
                path = [(k, l)]
                while good_values[k][l] == None: #and nxt[k][l] != None: #and \
                #    nxt[k][l] != v and nxt[k][l] != u:
                    k = nxt[k][l]
                    path.append((k, l))
                """
                print(path)
                print(edge)
                print(i, ' ,' , j)
                """
                if nxt[k][l] == None or good_values[k][l] == True:
                    #change = True
                    """
                    print("in true")
                    while len(path) != 0:
                        
                        print(path)
                        print()
                        value = d_w_current[k][l]
                        k, l = path.pop()
                        print(value)
                        print(d_w_current[k][l])
                        if d_w_current[k][l] != value + 1:
                            d_w_current[k][l] = value + 1
                        good_values[k][l] = True
                    print("end true")
                    
                    """
                    for k, l in path:
                        good_values[k][l] = True
                    
                elif nxt[k][l] == u or nxt[k][l] == v \
                    or good_values[k][l] == False:
                    for k, l in path:
                        good_values[k][l] = False
            if not good_values[i][j]:
                """
                change = True
                nbastar += 1
                """
                #astar qui s'arrete sur true ou le but
                x = a_star_without_edge(good_values, i, j, 
                                    graph, edge, d_w_current)
                assert(x != None)
    """            
    print(edge)
    for x in good_values:
        print(x)
    """                
    """
    for node in range(d[v]):
        if d[v][node] == u or d[u][node] == v:
            #le pcc passe par u,v
    """
    
    
    
    d_without[edge] = d_w_current
    
def test():
    g = ig.Graph(7)
    g.add_edges([(0, 1), (1, 2), (1, 3), (2, 6), (2, 3), (2, 5), (3, 4),
                 (4, 5), (5, 6)])
    
    for f in g.es:
        print(f.tuple)
    
    g = Grid(4, 4).graphe_m
    i = Instance(g, g, [], [], "")
    #g = i.deterministic_graph
    g = i.agent_graph
    t_s = t.perf_counter()
    d1 = floyd_warshall_full(g)
    t_e = t.perf_counter()
    print("fw base ", t_e - t_s)
    """
    print()
    for l in d1:
        print(l)
    print()
    """
    start = t.perf_counter()
    d2, n, dico = floyd_warshall(g)
    end_fw = t.perf_counter()
    print()
    for y in d2:
        print(y)
    print()
    for x in n:
        print(x)
    print("FW took ", end_fw - start)
    """
    print()
    
    for l in d2:
        print(l)
    print()
    for l in n:
        print(l)
    print()
    for edge in dico.keys():
        print("FOR EDGE ", edge)
        print()
        for t in dico[edge]:
            print(t)
        print()
    print()
    """
    x = a_star_pen(g, 0, 8, dico, d2)
    end = t.perf_counter()
    print("it tooks ", end - start)
    print(x)
    print()

    for edge in g.es:
        print(edge.tuple, " p = ", edge["proba"])
    print()  

    i.print_grid()

    """ 
    for k in range(len(d1)):
        for x, y in zip(d1[k],d2[k]):
            assert(x == y)
    print()
    print(g)
    """
    g_t = create_g_t(g, n, 8)
    print(g_t)
    
def test_fw_fwBis():
    g = Grid(4, 4).graphe_m
    i = Instance(g, g, [], [], "")
    g = i.agent_graph
    t_s = t.perf_counter()
    d1, n1, dico1 = floyd_warshall(g)
    t_e = t.perf_counter()
    print("fw base ", t_e - t_s)
    
    t_s = t.perf_counter()
    d2, n2, dico2 = floyd_warshall_bis(g)
    t_e = t.perf_counter()
    print("fw bis ", t_e - t_s)
    
    for c1, c2 in zip(d1, d2):
        for x1, x2 in zip(c1, c2):
            assert(x1 == x2)
            
    for c1, c2 in zip(n1, n2):
        for x1, x2 in zip(c1, c2):
            assert(x1 == x2)
    
    for k1 in dico1.keys():
        for c1, c2 in zip(dico1[k1], dico2[k1]):
            for x1, x2 in zip(c1, c2):
                if(x1 != x2):
                    print("edge : ", k1)
                    print( x1, "    ", x2)
                    print_dict(i, k1, dico1, dico2)
                assert(x1 == x2)
    print("SAME")

def print_dict(i, k1, dico1, dico2):
    print()
    i.print_grid()
    print()
    for i in range(len(dico1[k1])):
        for j in range(len(dico1[k1])):
            if dico1[k1][i][j] != dico2[k1][i][j]:
                print("at ", i, ", ", j, " : ", dico1[k1][i][j], " != ",
                      dico2[k1][i][j])
    print()
    print("AVEC ASTAR")
    for c1 in dico1[k1]:
        print(c1)
    print()
    print("AVEC FW ")
    print()
    for c1 in dico2[k1]:
        print(c1)

if __name__ == "__main__":
    """
    for i in range(100):
        test_fw_fwBis()
    """
    test_fw_fwBis()