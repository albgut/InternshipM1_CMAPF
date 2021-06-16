#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 15 12:09:52 2021

@author: algutier
"""

import time as t

from repeated_tateo import *
from dfs_tateo import *
from grid_generation import *
from configuration import *

def test_repeated_online():
    g = Grid(10, 10)
    movement_graph = g.graphe_m
    comm_graph = ig.Graph.Full(n=100)
    config_start = Configuration([0, 1, 2, 3, 4, 5])
    config_end = Configuration([99, 98, 97, 96, 95, 94])
    data1 = Instance(movement_graph, comm_graph, config_start, config_end, "astar")
    data2 = data1.copy()
    data3 = data1.copy()
    print("ALL DATA COMPUTED")
    t_start = t.perf_counter()
    path_repeated_tateo = repeated_tateo(data1)
    t_end = t.perf_counter()
    t_repeated_tateo = t_end - t_start
    print("result from repeated :")
    print_result(t_repeated_tateo, path_repeated_tateo)
    t_start = t.perf_counter()
    path_online = DFS_tateo(data2, True)
    t_end = t.perf_counter()
    t_online = t_end - t_start
    print("result from online :")
    print_result(t_online, path_online)
    assert(data1.deterministic_graph.get_edgelist() == data2.deterministic_graph.get_edgelist())
    assert(len(path_online) == len(path_repeated_tateo))
    print()
    data1.print_grid()

def print_result(time, path):
    print("time = ", time)
    if not isinstance(path, type(None)):
        print("len = ", len(path))
        for c in path:
            print(c)
    else:
        print("No path found")

if __name__ == "__main__":
    test_repeated_online()
    