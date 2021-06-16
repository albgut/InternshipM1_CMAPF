#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 10 18:19:57 2021

@author: algutier
"""

import igraph as ig
import random as r
import math as m

from configuration import *
from astar import *

"""
A class to represent all the data of the problem :
    - An agent graph which represent the graph with the knowledge of all
        the agents.
    - A deterministic representation of the graph without any randomness
        which is create at the beggining.
    - A communication graph for the connectivity.
    - The ending configuration.
    - The heuristic used.
"""
class Instance:
    
    def __init__(self, movement_graph, comm_graph, 
                 config_start, config_end, heuristic):
        self.make_proba(movement_graph)
        self.agent_graph = movement_graph.copy()
        self.deterministic_graph = self.generate_deterministic(
            movement_graph.copy())
        self.comm_graph = comm_graph
        self.config_start = config_start
        self.config_end = config_end
        self.heuristic = heuristic
        
        self.init_distance()
        
    """
    Return the sama object instance but with a different starting 
    configuration.
    """
    def new_start(self, new_start_config):
        self.config_start = new_start_config
        
    """
    Copy the values of the instance without generate randomness again.
    """
    def copy(self):
        new_data = Instance(ig.Graph(n=len(self.agent_graph.vs)), 
                        ig.Graph(), 
                        self.config_start, 
                        self.config_end, self.heuristic)
        new_data.agent_graph = self.agent_graph.copy()
        new_data.comm_graph = self.comm_graph.copy()
        new_data.deterministic_graph = self.deterministic_graph.copy()
        return new_data
        
    """
    Return True iff the edge between node_1 and node_2 is present in the 
    deterministic graph or if node_1 = node_2, 
    False otherwise.
    """
    def edge_present(self, node_1, node_2):
        present = self.deterministic_graph.get_eid(node_1, node_2, error=False)
        return node_1 == node_2 or not present == -1
        
    """
    Give the attribute "proba" to the edges of the graph if not present
    """  
    def make_proba(self, graphe):
        if not "proba" in graphe.es.attribute_names():
            graphe.es["proba"] = [0.0] * len(graphe.es)
        
    """
    Generate a deterministic graph which represent the real graph within the 
    agents will evolved in.
    """
    def generate_deterministic(self, graphe):
        edge_to_delete = []
        for edge in graphe.es:
            if r.random() < edge["proba"]:
                edge_to_delete.append(edge)
        graphe.delete_edges(edge_to_delete)
        return graphe
    
    """
    Build the matrix of all the distances between nodes in the graph.
    Initialize to None.
    """
    def init_distance(self):
        self.matrix_distance = [[None for i in range(len(self.agent_graph.vs))] 
                                for j in range(len(self.agent_graph.vs))]
    """
    Clear all the distance in the matrix distance
    """   
    def clear_distance(self):
        self.init_distance()
    
    """
    Get the distance in fonction of the attribute heuristic of the instance
    """
    def get_distance(self, node_1, node_2):
        #To compute only the upper part of the matrix
        if node_1 > node_2:
            tmp = node_1
            node_1 = node_2
            node_2 = tmp
        if isinstance(self.matrix_distance[node_1][node_2], type(None)):
            if self.heuristic == "euclide":
                self.matrix_distance[node_1][node_2] = self.euclidean_distance(
                    node_1, node_2)
            if self.heuristic == "astar":
                if a_star(self, node_1, node_2) == 0:
                    return m.inf
        return self.matrix_distance[node_1][node_2]
    
    """
    Add a new distance to the instance. Used in astar algorithm.
    """
    def add_distance(self, node_1, node_2, cost):
        #To compute only the upper part of the matrix
        if node_1 > node_2:
            tmp = node_1
            node_1 = node_2
            node_2 = tmp 
        self.matrix_distance[node_1][node_2] = cost
    
    """
    Return a position in the form (x,y) from a node in the graph
    """
    def get_position(self, node):
        x = self.agent_graph.vs[node]["x_coord"]
        y = self.agent_graph.vs[node]["y_coord"]
        return x, y

    """
    Utilitary functions
    """
    
    """
    Print a squared grid from the deterministic graph in the terminal.
    Do not use it with other graphs
    """
    def print_grid(self):
        nb_squared = len(self.deterministic_graph.vs)
        h = int(m.sqrt(len(self.deterministic_graph.vs)))
        tmp = nb_squared - 1
        taille_max = len(str(tmp))
        for j in range(2 * h -1):
            line = ""
            for i in range(2 * h - 1):
                if j % 2 == 0:
                    if i % 2 == 0:
                        index_node = int(j/2) * h + int(i/2)
                        s_node = str(index_node)
                        while len(s_node) < taille_max:
                            s_node += " "
                        line += s_node
                    else:
                        node_1 = int(j/2) * h + int((i - 1)/2)
                        node_2 = node_1 + 1
                        if self.edge_present(node_1, node_2):
                            line += "--"
                        else:
                            line += "  "
                else:
                    if i % 2 == 0:
                        node_1 = int((j - 1) / 2) * h + int(i / 2)
                        node_2 = int((j + 1) / 2) * h + int(i / 2)
                        if self.edge_present(node_1, node_2):
                            s = "|"
                        else:
                            s = " "
                        while len(s) < taille_max:
                            s += " "
                        line += s
                    else:
                        node_1 = int((j - 1) / 2) * h + int((i - 1) / 2)
                        node_2 = int((j + 1) / 2) * h + int((i + 1) / 2)
                        if self.edge_present(node_1, node_2):
                            node_1 = int((j - 1) / 2) * h + int((i + 1) / 2)
                            node_2 = int((j + 1) / 2) * h + int((i - 1) / 2)
                            if self.edge_present(node_1, node_2):
                                s = "X "
                            else:
                                s = "\\ "
                        else:
                            node_1 = int((j - 1) / 2) * h + int((i + 1) / 2)
                            node_2 = int((j + 1) / 2) * h + int((i - 1) / 2)
                            if self.edge_present(node_1, node_2):
                                s = "/ "
                            else:
                                s = "  "
                        line += s
            print(line)
    
    """
    Return the euclidean distance between two point.
    pre two node index from the graph
    post euclidean distance
    """
    def euclidean_distance(self, node_1, node_2):
        x_a1, y_a1 = self.get_position(node_1)
        x_a2, y_a2 = self.get_position(node_2)
        
        return m.sqrt((x_a1 - x_a2)*(x_a1 - x_a2) + (y_a1 - y_a2)*(y_a1 - y_a2))

    """
    Return the manhattan distance between two point.
    pre two node index from the graph
    post manhattan distance
    """
    def manhattan_distance(self, node_1, node_2):
        x_a1, y_a1 = self.get_position(node_1)
        x_a2, y_a2 = self.get_position(node_2)
        
        return abs(x_a1 - x_a2) + abs(y_a1 - y_a2)
        
if __name__ == "__main__":
    g_m = ig.Graph([(0, 1), (0, 3), (0, 2), (1, 3), (2, 4)])
    g_m.vs["x_coord"] = [0, 1, 0, 1, 0]
    g_m.vs["y_coord"] = [0, 0, 1, 1, 2]
    g_m.es["proba"] = [0.0, 0.7, 0.7, 0.0, 0.5]
    g_c = ig.Graph.Full(n=4)
    config_start = Configuration([1, 2])
    config_end = Configuration([0, 3])
    heuristic = "astar"
    d = Instance(g_m, g_c, config_start, config_end, heuristic)
    print("graph : ", d.deterministic_graph)
    print(g_m)
    print(d.get_distance(3, 2))
    print(d.get_distance(3, 0))
    print(d.get_distance(0, 3))
    print(d.matrix_distance)
    print(d.edge_present(0, 3))