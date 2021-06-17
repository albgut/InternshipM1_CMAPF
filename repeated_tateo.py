#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 14 15:51:37 2021

@author: algutier
"""

import igraph as ig

from dfs_tateo import *
from data_graph import *
from configuration import *
from grid_generation import *

"""
Apply tateo until the path computed is blocked by a stochastic edge. 
Restart if blocked.
"""

def repeated_tateo(data):
    path = DFS_tateo(data, False)
    if isinstance(path, type(None)):
        #No path found
        return None
    final_path = []
    block = False
    
    while True:
        current_config = path[0]
        next_config = path[1]
        if update_graph(data, current_config, next_config):
            block = True
        path.pop(0)
        final_path.append(current_config.copy())
        if next_config.same(data.config_end) or block:
            break
        
    if block:
        new_start_config = final_path.pop(-1)
        data.new_start(new_start_config)
        next_path = repeated_tateo(data)
        if isinstance(next_path, type(None)):
            return None
        else:
            return final_path + next_path
    else :
        final_path.append(data.config_end)
        return final_path
    
"""
Update the knowledge of the graph for the agent from a current configuration
The incomming edges that are not present in the deterministic graph are deleted
return true if the current path followed is blocked, false otherwise
"""            
def update_graph(data, current_config, next_config):
    graph_change = False
    path_block = False
    for agent in range(current_config.nb_agent):
        node = current_config.get_agent_pos(agent)
        edge_to_delete = []
        #The node of the same agent at the next configuration could be deleted 
        #before : have to verify.
        next_node = next_config.get_agent_pos(agent)
        found_next_position = (node == next_node)
        for neighbor in data.agent_graph.neighbors(node):
            if next_node == neighbor:
                found_next_position = True
            if not data.edge_present(node, neighbor):
                graph_change = True
                edge = data.agent_graph.get_eid(node, neighbor)
                edge_to_delete.append(edge)
                if is_in_path(node, neighbor, current_config, next_config):
                    path_block = True
        data.agent_graph.delete_edges(edge_to_delete)
        if not found_next_position:
            path_block = True
    if graph_change:
        data.clear_distance()
    return path_block
    
def is_in_path(node, neighbor, current_config, next_config):
    for agent in range(current_config.nb_agent):
        if current_config.get_agent_pos(agent) == node and \
            next_config.get_agent_pos(agent) == neighbor:
                return True
    return False

if __name__ == "__main__":
    g_m = Grid(10, 10)
    g_c = ig.Graph.Full(n=100)

    data = Instance(g_m.graphe_m, g_c, Configuration([2, 3]), 
                Configuration([94, 96]), "astar")
    path = repeated_tateo(data)
    print(data.deterministic_graph)
    if not isinstance(path, type(None)):
        for c in path:
            print(c)
    