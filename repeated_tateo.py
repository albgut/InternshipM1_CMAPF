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
        if update_graph(data, current_config):
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
"""            
def update_graph(data, current_config):
    graph_change = False
    for agent in range(current_config.nb_agent):
        node = current_config.get_agent_pos(agent)
        edge_to_delete = []
        for neighbor in data.agent_graph.neighbors(node):
            if not data.edge_present(node, neighbor):
                graph_change = True
                edge_to_delete.append(data.agent_graph.get_eid(node, neighbor))
        data.agent_graph.delete_edges(edge_to_delete)
    if graph_change:
        data.clear_distance()
    return graph_change

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
    