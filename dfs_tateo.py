#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 13 10:53:30 2021

@author: algutier
"""

import time as t
from heapq import *

from closed_Tree import *
from instance import *
from configuration import *
from heap_item import *


def DFS_tateo(instance, online):
    """
    Return a path according to the information in instance.

    Parameters
    ----------
    instance : Instance
        An object with all informations about the instance.
    online : boolean
        True for execute the online_tateo to a stochastic graph,
        False for execute the regular tateo on a deterministic graph.

    Returns
    -------
    None :
        If no path is found.
    path : Array<Configuration>
        A connected path from starting configuration to the target
        configuration.
    """
    
    start_time = t.perf_counter()
    path = [instance.config_start]
    closed = Closed_tree()
    while not path == [] and not time_out(start_time):
        current_config = path[-1]
        if online:
            update_graph(instance, current_config, closed)
        closed.add_configuration(current_config)
        if current_config.same(instance.config_end):
            return path
        next_config = find_best_child(instance, current_config, closed, 
                                      start_time)
        if not next_config.is_empty():
            path.append(next_config)
        else:
            path.pop()
    print("Cannot find a path")
    return None


def find_best_child(instance, current_config, closed, start_time):
    """
    Find the best next configuration after the current one.

    Parameters
    ----------
    instance : Instance
        An object with all informations about the instance.
    current_config : Configuration
        The current configuration.
    closed : Closed_tree
        An object which stores all the previous configurations explored.
    start_time : float
        The time (perf_counter) which the algorithm dfs start. Used to stop the
        algorithm if it take too long.

    Returns
    -------
    config : Configuration
        The next best configuration from the current one. If there is no next 
        configuration, it will be an empty configuration.

    """
    """
    print("NEW FBC")
    print("current config = ", current_config)
    """
    nb_total_agents = instance.config_start.nb_agent
    heap = []
    partial_config = Configuration([])
    h_cost = compute_h(instance, current_config, partial_config)
    #heap item structure is : ((h + g, -g, config), (g, h, config))
    item = Heap_item((h_cost, 0, partial_config), 
                     (0, h_cost, partial_config.copy())
                     )
    heappush(heap, item)
    while not heap == [] and not time_out(start_time):
        next_partial = heappop(heap)
        (current_g, _, partial_config) = next_partial.item
        num_agent = partial_config.nb_agent
        if num_agent == nb_total_agents:
            if isConnected(instance, partial_config) \
                and not closed.is_in(partial_config) \
                and not partial_config.same(current_config):
                    
                    """
                    print("Best result = ", next_partial.value[0], 
                          ", g = ", next_partial.value[1], 
                          ", h = ", next_partial.item[1],
                          ", config = ", partial_config)
                    for c in heap:
                        print("result = ", c.value[0],
                              ", g = ", c.value[1], 
                              ", h = ", c.item[1],
                              ", config = ", c.item[2])
                    """
                    
                    return partial_config
            continue
        else:
            successors = get_successors(instance, 
                                        current_config.get_agent_pos(num_agent)
                                        )
            for node in successors:
                new_config = partial_config.copy()
                new_config.add_agent(node)
                #g_cost = current_g + instance.euclidean_distance(
                #    current_config.get_agent_pos(num_agent), node)
                g_cost = current_g + compute_distance(
                    current_config.get_agent_pos(num_agent), node)
                h_cost = compute_h(instance, current_config, new_config)
                if not h_cost == m.inf and not g_cost == m.inf:
                    item = Heap_item((h_cost + g_cost, - g_cost, 
                                      new_config.copy()), 
                                     (g_cost, h_cost, new_config.copy()))
                    heappush(heap, item)
    return Configuration([])

"""
def find_best_child_avt(instance, current_config, closed, start_time):
    nb_total_agents = instance.config_start.nb_agent
    heap = []
    partial_config = Configuration([])
    h_cost = compute_h(instance, current_config, partial_config)
    #heap item structure is : ((g + h, |config|), (g, h, config))
    item = Heap_item((h_cost , 0), 
                     (0, h_cost, partial_config.copy())
                     )
    heappush(heap, item)
    while not heap == [] and not time_out(start_time):
        (current_g, _, partial_config) = heappop(heap).item
        num_agent = partial_config.nb_agent
        if num_agent == nb_total_agents:
            if isConnected(instance, partial_config) \
                and not closed.is_in(partial_config) \
                and not partial_config.same(current_config):
                    return partial_config
            continue
        else:
            successors = get_successors(instance, 
                                        current_config.get_agent_pos(num_agent)
                                        )
            for node in successors:
                new_config = partial_config.copy()
                new_config.add_agent(node)
                g_cost = current_g + instance.get_distance(
                    current_config.get_agent_pos(num_agent), node)
                h_cost = compute_h(instance, current_config, new_config)
                item = Heap_item((g_cost + h_cost, new_config.nb_agent), 
                                 (g_cost, h_cost, new_config.copy()))
                heappush(heap, item)
    return Configuration([])
"""

def compute_distance(node1, node2):
    """
    Returns the unit distance between the nodes. 

    Parameters
    ----------
    node1 : int
        The index of the first node.
    node2 : int
        The index of the second node.

    Returns
    -------
    distance : int
        The unit distance between the nodes. 0 if node1 == node2, 1 otherwise.

    """
    if node1 == node2:
        return 0
    else:
        return 1

"""
Return all the next possible position from the current node
"""            
def get_successors(instance, current_node):
    """
    Returns all the next possible positions from the current node.

    Parameters
    ----------
    instance : Instance
        An object with all informations about the instance.
    current_node : int
        The index of the current node.

    Returns
    -------
    next_positions : Array<int>
        An array which contains all the index of the neighbors of current_node
        and current_node itself.

    """
    next_positions = instance.agent_graph.neighbors(current_node)
    next_positions.append(current_node)
    return next_positions

            
def update_graph(instance, current_config, closed):
    """
    Update the agent_graph in instance to reflect the knowledge of the agents.

    Parameters
    ----------
    instance : Instance
        An object with all informations about the instance.
    current_config : Configuration
        The current configuration.
    closed : Closed_tree
        An object which stores all the previous configurations explored.

    Returns
    -------
    None
        All the incomming/outcomming edges to/from the current_config which
        is not present in the deterministic graph in instance are deleted.

    """
    graph_change = False
    for agent in range(current_config.nb_agent):
        node = current_config.get_agent_pos(agent)
        edge_to_delete = []
        for neighbor in instance.agent_graph.neighbors(node):
            if not instance.edge_present(node, neighbor):
                graph_change = True
                edge_to_delete.append(instance.agent_graph.get_eid(node, 
                                                                   neighbor))
        instance.agent_graph.delete_edges(edge_to_delete)
    if graph_change:
        closed.clear()
        instance.clear_distance()


def time_out(start):
    """
    Compute the current time elapsed from start.
    Use to stop the program if it take to many times to compute.

    Parameters
    ----------
    start : float
        The time (perf_counter) which the algorithm dfs start.

    Returns
    -------
    bool
        True if the time elapsed is greater than 60, False otherwise.

    """
    current_time = t.perf_counter() - start
    if current_time > 60:
        return True
    else:
        return False
    

def compute_h(instance, current_config, partial_config):
    """
    Compute the distance between the current partial configuration 
    and the goal ; the h cost of the partial configuration.

    Parameters
    ----------
    instance : Instance
        An object with all informations about the instance.
    current_config : Configuration
        The current configuration. Used to complete the partial configuration.
    partial_config : Configuration
        A partial configuration.

    Returns
    -------
    h_cost : float
        The h cost of the partial configuration.

    """
    if instance.heuristic == "HOP":
        return hop_h_cost(instance, current_config, partial_config)
    h_cost = 0
    for num_agent in range(partial_config.nb_agent):
        agent_current_node = partial_config.get_agent_pos(num_agent)
        agent_goal_node = instance.config_end.get_agent_pos(num_agent)
        h_cost += instance.get_distance(agent_current_node, 
                                        agent_goal_node, num_agent)
    for num_agent in range(partial_config.nb_agent, current_config.nb_agent):
        agent_current_node = current_config.get_agent_pos(num_agent)
        agent_goal_node = instance.config_end.get_agent_pos(num_agent)
        h_cost += instance.get_distance(agent_current_node, 
                                        agent_goal_node, num_agent)
    return h_cost

def hop_h_cost(instance, current_config, partial_config):
    """
    TODO
    """
    nb_rollout = 10
    final_h_cost = 0
    #print("\nnew HOP\n")
    for i in range(nb_rollout):
        new_inst = instance.sample()
        #print(new_inst.agent_graph.es["proba"])
        h_cost = 0
        for num_agent in range(partial_config.nb_agent):
            agent_current_node = partial_config.get_agent_pos(num_agent)
            agent_goal_node = new_inst.config_end.get_agent_pos(num_agent)
            h_cost += new_inst.get_distance(agent_current_node, 
                                            agent_goal_node, num_agent)
        for num_agent in range(partial_config.nb_agent, 
                               current_config.nb_agent):
            agent_current_node = current_config.get_agent_pos(num_agent)
            agent_goal_node = new_inst.config_end.get_agent_pos(num_agent)
            h_cost += new_inst.get_distance(agent_current_node, 
                                            agent_goal_node, num_agent)
        #print(h_cost)
        final_h_cost += h_cost
    return final_h_cost / nb_rollout
        

def isConnected(instance, config):
    """
    Verify if all the agent in the configuration are connected.

    Parameters
    ----------
    instance : Instance
        An object with all informations about the instance.
    config : Configuration
        The configuration to verify.

    Returns
    -------
    bool
        True if the configuration is connected, False otherwise.

    """
    stack = [0]
    agent = [False] * instance.config_start.nb_agent
    agent[0] = True
    count = 1
    while not len(stack) == 0:
        num_agent = stack.pop()
        for neighbor in instance.comm_graph.neighbors(
                config.get_agent_pos(num_agent)):
            for i in range(config.nb_agent):
                if config.get_agent_pos(i) == neighbor and not agent[i]:
                    agent[i] = True
                    count += 1
                    stack.append(i)
    return count == config.nb_agent


def to_list_agent(path):
    """
    Change the path from a list of Configuration to a list of paths, 
    one for each agent.

    Parameters
    ----------
    path : Array<Configuration>
        The path to change.

    Returns
    -------
    path_agent : Array<Array<int>>
        The list of the paths.

    """
    path_agent = []
    for i in range(path[0].nb_agent):
        path_agent.append([])
    for c in path:
        for agent in range(c.nb_agent):
            path_agent[agent].append(c.get_agent_pos(agent))
    return path_agent
    
if __name__ == "__main__":
    g_c = ig.Graph.Full(n=14)
    g_c = ig.Graph()
    g_c.add_vertices(14)
    g_m = ig.Graph.Full(n=14)
    instance = Instance(g_m, g_c, Configuration([1,2]), 
                        Configuration([10, 13]), "astar")
    config = Configuration([4, 9])
    print(isConnected(instance, config))
    
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