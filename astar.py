import igraph as ig
from heapq import *

import data_graph

"""
Update the distance in the matrix distance in the data object.
return 1 if a path was found 0 otherwise.
"""
def a_star(data, start_node, end_node):
    graphe = data.agent_graph
    open_heap = []
    closed_set = set()
    #((f,h,g),node)
    h_cost = data.euclidean_distance(start_node, end_node)
    heappush(open_heap, ((h_cost, h_cost, 0),start_node))
    while not open_heap == []:
        (f, h, g), node = heappop(open_heap)
        closed_set.add(node)
        data.add_distance(start_node, node, g)
        if node == end_node:
            return 1
        ens_neighbors = graphe.neighbors(node)
        for neighbor in ens_neighbors:
            edge_id = graphe.get_eid(node, neighbor)
            if not graphe.es[edge_id]["proba"] == 1:
                if not neighbor in closed_set:
                    dist_curr_to_neigh = data.euclidean_distance(node, 
                                                                 neighbor)
                    new_g_cost = g + dist_curr_to_neigh
                    new_h_cost = data.euclidean_distance(node, end_node)
                    new_f_cost = new_g_cost + new_h_cost
                    open_neighbor = find_in_open(open_heap, neighbor)
                    if not open_neighbor == None:
                        (old_f, old_h, old_g), _ = open_neighbor
                        if new_f_cost < old_f or (
                                new_f_cost == old_f and 
                                new_h_cost < old_h):
                            open_neighbor = ((new_g_cost + new_h_cost,
                                             new_h_cost, new_g_cost), neighbor)
                    else:
                        open_neighbor = ((new_g_cost + new_h_cost,
                                         new_h_cost, new_g_cost), neighbor)
                    heappush(open_heap, open_neighbor)
    return 0
    
def find_in_open(open_heap, node):
    for i in range(len(open_heap)):
        if open_heap[i][1] == node:
            return open_heap.pop(i)
    return None
