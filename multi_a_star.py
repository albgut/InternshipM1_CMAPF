#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul  9 14:48:42 2021

@author: algutier
"""

import igraph as ig
from heapq import *
import math as m
import time as t

from grid_generation import *

from instance import *

def a_star(instance, start_node, end_node, reserved, beggin=False):
    """
    TODO
    """
    graphe = instance.agent_graph
    open_heap = []
    closed_set = set()
    previous_node = dict()
    #((f,h,g),config)
    h_cost = instance.manhattan_distance(start_node, end_node)
    heappush(open_heap, ((h_cost, h_cost, 0), start_node))
    previous_node[start_node] = None
    time = 0
    while not open_heap == []:
        time += 1
        (f, h, g), current_node = heappop(open_heap)
        closed_set.add(current_node)
        if current_node == end_node:
            return build_path(previous_node, end_node)
        l_nodes = next_nodes_ca(instance, current_node, time, reserved, beggin)
        for next_node in l_nodes:
            if not next_node in closed_set:
                new_g_cost = g + instance.manhattan_distance(current_node, 
                                                             next_node)
                new_h_cost = instance.manhattan_distance(next_node, end_node)
                new_f_cost = new_g_cost + new_h_cost
                open_neighbor = find_in_open(open_heap, next_node)
                if not open_neighbor == None:
                    (old_f, old_h, old_g), _ = open_neighbor
                    if new_g_cost < old_g:
                        open_neighbor = ((new_f_cost,
                                          new_h_cost, new_g_cost), next_node)
                else:
                    open_neighbor = ((new_f_cost,
                                      new_h_cost, new_g_cost), next_node)
                previous_node[next_node] = current_node
                heappush(open_heap, open_neighbor)
    print("Cannot find a path")
    return None


def a_star_multi(instance):
    """
    Generate the shortest path according to the information in the instance 
    using the A* algorithm.

    Parameters
    ----------
    instance : Instance
        The instance of the problem.

    Returns
    -------
    path : List<Configuration>
        The shortest path according to the instance information.

    """
    
    graphe = instance.agent_graph
    open_heap = []
    closed_set = set()
    previous_config = dict()
    #((f,h,g),config)
    h_cost = compute_distance(instance, instance.config_start, 
                              instance.config_end)
    heappush(open_heap, ((h_cost, h_cost, 0), config_start))
    previous_config[config_start] = None
    while not open_heap == []:
        (f, h, g), current_config = heappop(open_heap)
        closed_set.add(current_config)
        if not isConnected(instance, current_config):
            continue
        if current_config == config_end:
            return build_path(previous_config, config_end)
        l_configs = next_configs(graphe, current_config)
        for next_config in l_configs:
            if not next_config in closed_set:
                new_g_cost = g + cost_step(instance, current_config, 
                                           next_config)
                new_h_cost = compute_distance(instance, next_config, 
                                              instance.config_end)
                new_f_cost = new_g_cost + new_h_cost
                open_neighbor = find_in_open(open_heap, next_config)
                if not open_neighbor == None:
                    (old_f, old_h, old_g), _ = open_neighbor
                    if new_g_cost < old_g:
                        open_neighbor = ((new_f_cost,
                                          new_h_cost, new_g_cost), next_config)
                else:
                    open_neighbor = ((new_f_cost,
                                      new_h_cost, new_g_cost), next_config)
                previous_config[next_config] = current_config
                heappush(open_heap, open_neighbor)
    print("Cannot find a path")
    return None
                
                
def find_in_open(open_heap, config):
    """
    Find a configuration into an heap, pop it and return it with its values
    f, g and h.

    Parameters
    ----------
    open_heap : Heap<(float,float,float), Configuration> 
                (f,g,h), node
        The heap with the node currently in treatment during the a star.
    config : Configuration
        The configuration to find in the heap.

    Returns
    -------
    item_heap : (float,float,float), Configuration
                (f,g,h), node
        The item in the heap which contains the configuration and its 
        values f, g and h.
    """
    
    for i in range(len(open_heap)):
        (_, config_heap) = open_heap[i]
        if config_heap == config:
            item_heap = open_heap.pop(i)
            return item_heap
    return None


def cost_step(instance, current_config, next_config):
    """
    Return the cost from the current configuration to the next possible one.

    Parameters
    ----------
    instance : Instance
        The instance of the problem.
    current_config : Configuration
        The current configuration.
    next_config : Configuration
        The next configuration. Have to be a possible next configuration 
        from the current configuration.

    Returns
    -------
    cost : float
        The cost of moving from the current configuration to the next one.

    """
    cost = 0
    for agent in range(current_config.nb_agent):
        cost += instance.manhattan_distance(current_config.get_agent_pos(agent), 
                                            next_config.get_agent_pos(agent))
    return cost

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
        ens = instance.comm_graph.neighbors(config.get_agent_pos(num_agent))
        ens.append(config.get_agent_pos(num_agent))
        for neighbor in ens:
            for i in range(config.nb_agent):
                if config.get_agent_pos(i) == neighbor and not agent[i]:
                    agent[i] = True
                    count += 1
                    stack.append(i)
    return count == config.nb_agent

def next_configs(graph, config):
    """
    Generate the list of all the possible next configurations from the 
    current one.

    Parameters
    ----------
    graph : Graph
        The graph to explore.
    config : Configuration
        The configuration from which you want the next configurations.

    Returns
    -------
    list_next_config : List<Configuration>
        The list of the next possible configurations.

    """
    global list_next_config, current_config, start_config, g
    list_next_config = []
    g = graph
    start_config = config.copy()
    current_config = Configuration([0] * config.nb_agent)
    aux_configs(0)
    return list_next_config

def aux_configs(i):
    """
    An auxilary function used for the backtracking.

    Parameters
    ----------
    i : int
        The indax of the current agent.

    Returns
    -------
    None.

    """
    global list_next_config, current_config, start_config, g
    nb_agent = start_config.nb_agent
    ens_neighbors = g.neighbors(start_config.get_agent_pos(i))
    ens_neighbors.append(start_config.get_agent_pos(i))
    for neighbour in ens_neighbors:
        current_config.change_pos(i, neighbour)
        if i == nb_agent - 1 and not current_config == start_config:
            list_next_config.append(current_config.copy())
        elif i < nb_agent - 1:
            aux_configs(i + 1)
        
        
def build_path(previous_config, config_end):
    """
    Returns the shortest path from the starting configuration to the ending 
    configuration.

    Parameters
    ----------
    previous_config : Dict<(Configuration, Configuration)>
        A dictionnary of previous configurations.
    config_end : Configuration
        The ending configuration.

    Returns
    -------
    path : List<Configuration>
        The shortest path from starting configuration and anding configuration.

    """
    current_config = config_end
    path = [config_end]
    while True:
        current_config = previous_config[current_config]
        if isinstance(current_config, type(None)):
            break
        else:
            path.insert(0, current_config)
    return path
        
def compute_distance(instance, config_start, config_end):
    """
    Compute the cost from the starting configuration to the 
    ending configuration, using the sum on all the agents cost in the 
    configurations.

    Parameters
    ----------
    config_start : Configuration
        The starting configuration.
    config_end : Configuration
        The ending configuration.

    Returns
    -------
    cost : float
        The estimate cost from starting configuration to ending configuration.

    """
    cost = 0
    for agent in range(config_start.nb_agent):
        node_start = config_start.get_agent_pos(agent)
        node_end = config_end.get_agent_pos(agent)
        cost += instance.manhattan_distance(node_start, node_end)
    return cost

def h_c_a_star(instance):
    """
    TODO
    """
    reserved = set()
    path = [0] * instance.config_start.nb_agent
    for agent in range(instance.config_start.nb_agent):
        path[agent] = plan_path(instance, agent, reserved)
    return path

def plan_path(instance, agent, reserved):
    """
    TODO
    """
    if agent == 0:
        path = a_star(instance, instance.config_start.get_agent_pos(agent),
                      instance.config_end.get_agent_pos(agent), reserved, True)
    else:
        path = a_star(instance, instance.config_start.get_agent_pos(agent),
                      instance.config_end.get_agent_pos(agent), reserved)
    for i in range(len(path)):
        reserved.add((path[i], i))
        
    print("\nNEW RESERVED :\n")
    print(reserved, "\n")
    return path
        
def next_nodes_ca(instance, current_node, time, reserved, beggin):
    """
    TODO
    """
    ens = instance.agent_graph.neighbors(current_node)
    ens.append(current_node)
    if beggin:
        return ens
    else:
        for node in ens:
            connected = False
            for comm_node in instance.comm_graph.neighbors(node):
                if (comm_node, time) in reserved:
                    connected = True
                    break
            if not connected:
                ens.remove(node)
        return ens

if __name__ == "__main__":
    print("\tTEST FOR BT :\n")
    g = Grid(10, 10)
    graph = g.graphe_m
    config_start = Configuration([0, 1])
    config_end = Configuration([99, 98])
    i = Instance(graph, graph, config_start, config_end, "astar")
    i.print_grid()
    print()
    res = next_configs(graph, Configuration([32, 33]))
    v = 0
    for c in res :
        v += 1
        print(c)
    print(v)
    print(9**2)
    print("\n\tTEST FOR DICT HASH :\n")
    d = dict()
    d[Configuration([1, 2])] = Configuration([8, 9])
    for key in d.keys():
        print(key, ", ", d[key])
        
    print("\n\tTEST ASTAR MULTI :\n")
    t_s = t.perf_counter()
    l = a_star_multi(i)
    t_end = t.perf_counter()
    print("it tooks ", t_end - t_s, "s.")
    if not isinstance(l, type(None)):
        for c in l:
            print(c)
    
    print("\n\tTEST HCASTAR :\n")
    t_s = t.perf_counter()
    l = h_c_a_star(i)
    t_end = t.perf_counter()
    print("it tooks ", t_end - t_s, "s.")
    if not isinstance(l, type(None)):
        for c in l:
            print(c)