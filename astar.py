import igraph as ig
from heapq import *

from instance import *


def a_star(instance, start_node, end_node, agent=None):
    """
    Compute the cost of the shortest path from start_node to end_node 
    using a star algorithm.

    Parameters
    ----------
    instance : Instance
        An object with all informations about the instance.
    start_node : int
        The index of the starting node.
    end_node : int
        The index of the goal.
    agent : int
        The index of the current agent. Used for penalty based algorithm.

    Returns
    -------
    int
        The cost of the shortest path is added to the matrix_distance 
        attribute of the instance. Returns -1 if the algorithm can not 
        find a path or 1 (or the cost of the path if the heuristic is penalty)
        if it found one.

    """
    graphe = instance.agent_graph
    open_heap = []
    closed_set = set()
    #((f,h,g),node)
    h_cost = instance.euclidean_distance(start_node, end_node)
    heappush(open_heap, ((h_cost, h_cost, 0),start_node))
    while not open_heap == []:
        (f, h, g), node = heappop(open_heap)
        closed_set.add(node)
        if agent == None:
            instance.add_distance(start_node, node, g)
        if node == end_node:
            if agent == None:
                return 1
            else:
                return g
        ens_neighbors = graphe.neighbors(node)
        for neighbor in ens_neighbors:
            edge_id = graphe.get_eid(node, neighbor)
            #if not graphe.es[edge_id]["proba"] == 1:
            if not neighbor in closed_set:
                dist_curr_to_neigh = instance.euclidean_distance(node, 
                                                             neighbor)
                if instance.heuristic == "penalty":
                    dist_curr_to_neigh += instance.penalty_func(node, neighbor,
                                                                agent)
                new_g_cost = g + dist_curr_to_neigh
                #new_g_cost = g + 1
                new_h_cost = instance.euclidean_distance(neighbor, end_node)
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
    return -1
    
def find_in_open(open_heap, node):
    """
    Find a node into an heap, pop it and return it with its values f, g and h.

    Parameters
    ----------
    open_heap : Heap<(float,float,float), int> 
                (f,g,h), node
        The heap with the node currently in treatment during the a star.
    node : int
        The node to find in the heap.

    Returns
    -------
    item_heap : (float,float,float), int
                (f,g,h), node
        The item in the heap which contains the node and its values f, g and h.

    """
    for i in range(len(open_heap)):
        (_, node_heap) = open_heap[i]
        if node_heap == node:
            item_heap = open_heap.pop(i)
            return item_heap
    return None
