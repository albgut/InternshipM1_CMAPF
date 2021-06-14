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

"""
Apply tateo until the path computed is blocked by a stochastic edge. 
Restart if blocked.
"""

def repeated_tateo(data):
    path = dfs_tateo(data)
    if isinstance(path, type(None)):
        #No path found
        return None
    
    final_path = []
    
    while True:
        next_config = path[0]
        for agent in range(next_config.nb_agent):
            
            
            
            next_node = path[0].get_agent_pos(agent)
            next_config.append(next_node)
            next_edges = info_graph.graph.vs[next_node].in_edges()
            


def multi_heur(info_graph, config_start, config_end, heuristic):
    
    #info_graph = Info_graph(graphe, heuristic)
    """
    nb_agent = len(config_start)
    if heuristic == "OMT":
        path = OMT(graphe, config_start, config_end)
    if heuristic == "HOP":
        path = HOP(nb_agent, graphe, config_start, config_end)
    """
    if heuristic == "HOP":
        test_multi_heuristic_HOP(info_graph.graph, config_start, config_end)
    
    nb_agent = len(config_start)
    
     
    opti_path = []
    noblock = True
    
    if path == []:
        return []
    
    while True:
        #the next configuration in the final path
        new_config = []
        for agent in range(nb_agent):
            next_node = path[0][agent]
            #move to the next node
            new_config.append(next_node)
            next_edges = info_graph.graph.vs[next_node].in_edges()
            for edge in next_edges:
                if info_graph.get_edge_present(edge.index):
                    info_graph.graph.es[edge.index]["deterministic"] = True
                    info_graph.graph.es[edge.index]["proba"] = 0
                else:
                    noblock = False
                    info_graph.graph.es[edge.index]["deterministic"] = True
                    info_graph.graph.es[edge.index]["proba"] = 1
        path.pop(0) 
        opti_path.append(new_config.copy())
        if not noblock or path[0] == config_end:
            break
    
    if not noblock:
        new_start_config = opti_path[len(opti_path) - 1]
        return opti_path + multi_heur(info_graph, new_start_config, 
                                      config_end,heuristic)
    else :
        opti_path.append(config_end)
        return opti_path

if __name__ == "__main__":
    