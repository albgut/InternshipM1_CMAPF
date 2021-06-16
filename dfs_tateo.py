#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 13 10:53:30 2021

@author: algutier
"""

import time as t
from heapq import *

from closed_Tree import *
from data_graph import *
from configuration import *
from heap_item import *

"""
Depth-First Search from tateo in partially known environment
data is a Instance object with all infos about the instance
"""
def DFS_tateo(data, online):
    start_time = t.perf_counter()
    path = [data.config_start]
    closed = Closed_tree()
    while not path == [] and not time_out(start_time):
        current_config = path[-1]
        if online:
            update_graph(data, current_config, closed)
        closed.add_configuration(current_config)
        if current_config.same(data.config_end):
            return path
        next_config = find_best_child(data, current_config, closed, start_time)
        if not next_config.is_empty():
            path.append(next_config)
        else:
            path.pop()
    print("Cannot find a path")
    return None

def find_best_child(data, current_config, closed, start_time):
    nb_total_agents = data.config_start.nb_agent
    heap = []
    partial_config = Configuration([])
    h_cost = compute_h(data, current_config, partial_config)
    #heap item structure is : ((h + g, -g, config), (g, h, config))
    #Used to store g and h and compare only with g + h
    item = Heap_item((h_cost, 0, partial_config), 
                     (0, h_cost, partial_config.copy())
                     )
    heappush(heap, item)
    while not heap == [] and not time_out(start_time):
        next_partial = heappop(heap)
        (current_g, _, partial_config) = next_partial.item
        num_agent = partial_config.nb_agent
        if num_agent == nb_total_agents:
            if isConnected(data, partial_config) \
                and not closed.is_in(partial_config) \
                and not partial_config.same(current_config):
                    return partial_config
            continue
        else:
            successors = get_successors(data, 
                                        current_config.get_agent_pos(num_agent)
                                        )
            for node in successors:
                new_config = partial_config.copy()
                new_config.add_agent(node)
                g_cost = current_g + data.euclidean_distance(
                    current_config.get_agent_pos(num_agent), node)
                h_cost = compute_h(data, current_config, new_config)
                if not h_cost == m.inf and not g_cost == m.inf:
                    item = Heap_item((h_cost + g_cost, -g_cost, new_config), 
                                     (g_cost, h_cost, new_config.copy()))
                    heappush(heap, item)
    return Configuration([])

"""
def find_best_child_avt(data, current_config, closed, start_time):
    nb_total_agents = data.config_start.nb_agent
    heap = []
    partial_config = Configuration([])
    h_cost = compute_h(data, current_config, partial_config)
    #heap item structure is : ((g + h, |config|), (g, h, config))
    #Used to store g and h and compare only with g + h
    item = Heap_item((h_cost , 0), 
                     (0, h_cost, partial_config.copy())
                     )
    heappush(heap, item)
    while not heap == [] and not time_out(start_time):
        (current_g, _, partial_config) = heappop(heap).item
        num_agent = partial_config.nb_agent
        if num_agent == nb_total_agents:
            if isConnected(data, partial_config) \
                and not closed.is_in(partial_config) \
                and not partial_config.same(current_config):
                    return partial_config
            continue
        else:
            successors = get_successors(data, 
                                        current_config.get_agent_pos(num_agent)
                                        )
            for node in successors:
                new_config = partial_config.copy()
                new_config.add_agent(node)
                g_cost = current_g + data.get_distance(
                    current_config.get_agent_pos(num_agent), node)
                h_cost = compute_h(data, current_config, new_config)
                item = Heap_item((g_cost + h_cost, new_config.nb_agent), 
                                 (g_cost, h_cost, new_config.copy()))
                heappush(heap, item)
    return Configuration([])
"""
"""
Return all the next possible position from the current node
"""            
def get_successors(data, current_node):
    next_positions = data.agent_graph.neighbors(current_node)
    next_positions.append(current_node)
    return next_positions


"""
Update the knowledge of the graph for the agent from a current configuration
The incomming edges that are not present in the deterministic graph are deleted
"""            
def update_graph(data, current_config, closed):
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
        closed.clear()
        data.clear_distance()


"""
Use to stop the program if it take to many times to compute.
"""
def time_out(start):
    current_time = t.perf_counter() - start
    if current_time > 60:
        return True
    else:
        return False
    
"""
Calculate the heuristic value which correspond to the distance from the 
positions of all the agents in a configuration to their respective goals.
TODO
pre 
post 
"""
def compute_h(data, current_config, partial_config):
    h_cost = 0
    for num_agent in range(partial_config.nb_agent):
        agent_current_node = partial_config.get_agent_pos(num_agent)
        agent_goal_node = data.config_end.get_agent_pos(num_agent)
        h_cost += data.get_distance(agent_current_node, agent_goal_node)
    for num_agent in range(partial_config.nb_agent, current_config.nb_agent):
        agent_current_node = current_config.get_agent_pos(num_agent)
        agent_goal_node = data.config_end.get_agent_pos(num_agent)
        h_cost += data.get_distance(agent_current_node, agent_goal_node)
    return h_cost

"""
Return True iff all the agents in the configuration are connected in the
communication graph
TODO
"""
def isConnected(data, config):
    stack = [0]
    agent = [False] * data.config_start.nb_agent
    agent[0] = True
    count = 1
    while not len(stack) == 0:
        num_agent = stack.pop()
        for neighbor in data.comm_graph.neighbors(
                config.get_agent_pos(num_agent)):
            for i in range(config.nb_agent):
                if config.get_agent_pos(i) == neighbor and not agent[i]:
                    agent[i] = True
                    count += 1
                    stack.append(i)
    return count == config.nb_agent

    
if __name__ == "__main__":
    g_c = ig.Graph.Full(n=14)
    g_c = ig.Graph()
    g_c.add_vertices(14)
    g_m = ig.Graph.Full(n=14)
    data = Instance(g_m, g_c, Configuration([1,2]), Configuration([10, 13]), "astar")
    config = Configuration([4, 9])
    print(isConnected(data, config))
    
    g_m = ig.Graph([(0, 1), (0, 3), (0, 2), (1, 3), (2, 4)])
    g_m.vs["x_coord"] = [0, 1, 0, 1, 0]
    g_m.vs["y_coord"] = [0, 0, 1, 1, 2]
    g_m.es["proba"] = [0.0, 0.7, 0.7, 0.0, 0.5]
    g_c = ig.Graph.Full(n=4)
    config_start = Configuration([1, 2])
    config_end = Configuration([0, 3])
    heuristic = "astar"
    d = Instance(g_m, g_c, config_start, config_end, heuristic)
    d.agent_graph.es[1]["proba"] = 1
    
    print("graph : ", d.deterministic_graph)
    print("graph : ", d.agent_graph, "\nPROBA \n", d.agent_graph.es["proba"])