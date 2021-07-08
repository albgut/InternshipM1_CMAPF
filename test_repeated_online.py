#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 15 12:09:52 2021

@author: algutier
"""

import time as t
import random as r

from repeated_tateo import *
from dfs_tateo import *
from grid_generation import *
from configuration import *
from astar import *


def test_repeated_online(grid_length, seed=r.randint(0, 10000)):
    """
    A test to to compare the time of execution and the length of the result 
    path between the online_tateo and repeated_tateo algorithm on a grid.

    Parameters
    ----------
    grid_length : int
        The length of the squared grid to create for the test.
    seed : int, optional
        A seed to generate the grid. The default is r.randint(0, 10000).

    Returns
    -------
    None.

    """
    #seed with different execution : 4x4,8175
    g = Grid(grid_length, grid_length, seed=seed)
    movement_graph = g.graphe_m
    nb_vertices = grid_length ** 2
    comm_graph = ig.Graph.Full(n=nb_vertices)
    config_start = Configuration([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
    #config_start = Configuration([0, 1, 2])
    
    config_end = Configuration([nb_vertices - 1, nb_vertices - 2, 
                                nb_vertices - 3, nb_vertices - 4, 
                                nb_vertices - 5, nb_vertices - 6,
                                nb_vertices - 7, nb_vertices - 8,
                                nb_vertices - 9, nb_vertices - 10,
                                nb_vertices - 11, nb_vertices - 12])
    """
    config_end = Configuration([nb_vertices - 1, nb_vertices - 2,
                                nb_vertices - 3])
    """
    instance1 = Instance(movement_graph, comm_graph, 
                     config_start, config_end, "astar", seed=seed)
    instance2 = instance1.copy()
    instance3 = instance1.copy()
    instance3.heuristic = "penalty"
    print("ALL DATA COMPUTED")
    t_start = t.perf_counter()
    path_repeated_tateo = repeated_tateo(instance1)
    t_end = t.perf_counter()
    t_repeated_tateo = t_end - t_start
    print("result from repeated :")
    print_result(t_repeated_tateo, path_repeated_tateo)
    t_start = t.perf_counter()
    path_online = DFS_tateo(instance2, True)
    t_end = t.perf_counter()
    t_online = t_end - t_start
    print("result from online :")
    print_result(t_online, path_online)
    
    
    t_start = t.perf_counter()
    path_dfs = DFS_tateo(instance3, True)
    t_end = t.perf_counter()
    t_penalty = t_end - t_start
    print("result from penalty :")
    print_result(t_penalty, path_dfs)
    
    #print(to_list_agent(path_dfs))
    
    
    assert(instance1.deterministic_graph.get_edgelist() == 
           instance2.deterministic_graph.get_edgelist())
    
    print()
    print("seed = ", instance1.seed)
    print()
    instance1.print_grid()
    #if not isinstance(path_repeated_tateo, type(None)) and \
    #    not isinstance(path_online, type(None)):
    #        assert(len(path_online) == len(path_repeated_tateo))
    print()
    min_time = min(t_online, t_penalty, t_repeated_tateo)
    
    s = "Min time = "
    if min_time == t_online:
        s += "online "
    if min_time == t_penalty:
        s += "penalty "
    if min_time == t_repeated_tateo:
        s += "repeated "
    print(s)
    
    s = "\nMin path = "
    min_path = min(len(path_dfs), len(path_online), 
                   len(path_repeated_tateo))
    if min_path == len(path_online):
        s += "online "
    if min_path == len(path_dfs):
        s += "penalty "
    if min_path == len(path_repeated_tateo):
        s += "repeated "
    print(s)

def print_result(time, path):
    """
    Print the time and the path.

    Parameters
    ----------
    time : float
        Time to print.
    path : Array<Configuration>
        The configuration to print.

    Returns
    -------
    None.

    """
    print("time = ", time)
    if not isinstance(path, type(None)):
        print("len = ", len(path))
        for c in path:
            print(c)
    else:
        print("No path found")

if __name__ == "__main__":
    test_repeated_online(10)