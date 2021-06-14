#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 13 18:12:31 2021

@author: algutier
"""

import os
import igraph as ig

from data_graph import *
from dfs_tateo import *
from configuration import *

"""
Treat all the files .exp in the path
"""
def treat_file_from_dir(path):
    l_file = os.listdir(path)
    nb_of_file = 0
    log_dir = "log" + path.split("/")[2][3:]
    for file_name in l_file:
        nb_of_file += 1
        print("TREAT FILE ", nb_of_file, " at ", t.localtime().tm_hour, " h ",
              t.localtime().tm_min, " min ", t.localtime().tm_sec, " s.")
        treat_file(path, file_name, log_dir)
        
"""
Treat a file .exp and put the result in the log directory
"""       
def treat_file(path, file_name, log_dir):
    print(file_name)
    file = open(path + "/" + file_name, "r")
    for line in file:
        parse = line.split(" ")
        if parse[0] == "phys_graph":
            g_m = ig.Graph.Read_GraphML("./graphML/" + parse[1].split("\n")[0])
        elif parse[0] == "comm_graph":
            g_c = ig.Graph.Read_GraphML("./graphML/" + parse[1].split("\n")[0])
        elif parse[0] == "start":
            l_config_start = []
            for i in range(1, len(parse) - 1):
                l_config_start.append(int(parse[i]))
            config_start = Configuration(l_config_start)
        elif parse[0] == "goal":
            l_config_end = []
            for i in range(1, len(parse) - 1):
                l_config_end.append(int(parse[i]))
            config_end = Configuration(l_config_end)
        else:
            print("error in file ")
            sys.exit()
    graph = (g_m, g_c)
    data = Data(g_m, g_c, config_start, config_end, "astar")
    time_start = t.perf_counter()
    res_path = DFS_tateo(data)
    time_end = t.perf_counter()
    log_file = open("./logs/" + log_dir + "/" + file_name.split(".")[0] + ".log", 'w')
    if time_end - time_start > 300:
        log_file.write("NO PATH FOUND : TIME OUT\n")
    else:
        for i in range(len(res_path)):
            log_file.write("s" + str(i))
            for j in range(res_path[i].nb_agent):
                log_file.write(" " + str(res_path[i].get_agent_pos(j)))
            log_file.write("\n")
    log_file.write("t " + str(time_end - time_start) + " s")
    log_file.close()

if __name__ == "__main__":
    """
    treat_file("./exp/exp_10_grid_25", 
               "b-w-open_uniform_grid_13_range_25_10_18.exp", 
               "log_10_grid_25")
    """
    treat_file_from_dir("./exp/exp_2_grid_25")
    