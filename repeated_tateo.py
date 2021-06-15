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
        for agent in range(next_config.nb_agent):
            if not data.edge_present(current_config.get_agent_pos(agent),
                                     next_config.get_agent_pos(agent)):
                block = True
                #We update the knowledge of the agents
                edge = data.agent_graph.get_eid(\
                    current_config.get_agent_pos(agent), 
                    next_config.get_agent_pos(agent))
                data.agent_graph.delete_edges(edge)
        path.pop(0)
        final_path.append(current_config.copy())
        if next_config.same(data.config_end) or block:
            break
        
    if block:
        new_start_config = final_path.pop(-1)
        data.new_start(new_start_config)
        next_path = repeated_tateo(data)
        if isinstance(path, type(None)):
            return None
        else:
            return final_path + next_path
    else :
        final_path.append(data.config_end)
        return final_path

if __name__ == "__main__":
    g_m = Grid(10, 10)
    g_c = ig.Graph.Full(n=100)

    data = Data(g_m.graphe_m, g_c, Configuration([2, 3]), 
                Configuration([94, 96]), "astar")
    path = repeated_tateo(data)
    print(data.deterministic_graph)
    if not isinstance(path, type(None)):
        for c in path:
            print(c)
    