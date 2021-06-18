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


class Instance:
    """
A class to represent all the data of the problem :
    
    - agent_graph : Graph
        An graph which represent the knowledge of all the agents.
        
    - deterministic_graph : Graph
        A deterministic representation of the graph without any randomness
        which is create at the beggining.
        
    - comm_graph : Graph
        A communication graph for the connectivity.
        
    - config_start : Configuration
        The starting configuration
        
    - config_end : Configuration
        The ending configuration.
        
    - heuristic : string
        The heuristic used.
    - seed : float
        A seed for randomness.
    """
    
    def __init__(self, movement_graph, comm_graph, config_start, config_end, 
                 heuristic, seed=r.randint(0, 10000)):
        """
        Create a new instance.

        Parameters
        ----------
        movement_graph : Graph
            The movement graph of the problem. Need to have the attributes 
            "x-coord" and "y-coord" on the vertices. A copy is created.
        comm_graph : Graph
            The communication graph of the problem.
        config_start : Configuration
            The starting configuration.
        config_end : Configuration
            The goal configuration.
        heuristic : string
            The heuristic to used choose from : 
                "astar", "euclide", "manhattan".
        seed : float, optional
            The seed of the randomness. The default is r.randint(0, 10000).

        Returns
        -------
        None.

        """
        r.seed(a=seed)
        self.seed = seed
        self.make_proba(movement_graph)
        self.agent_graph = movement_graph.copy()
        self.deterministic_graph = self.generate_deterministic(
            movement_graph.copy())
        self.comm_graph = comm_graph
        self.config_start = config_start
        self.config_end = config_end
        self.heuristic = heuristic
        
        self.init_distance()
        
    def new_start(self, new_start_config):
        """
        Change the starting configuration of the instance.

        Parameters
        ----------
        new_start_config : Configuration
            The new starting configuration.

        Returns
        -------
        None.

        """
        self.config_start = new_start_config
        
        
    def copy(self):
        """
        Copy the instance.

        Returns
        -------
        new_instance : Instance
            The new instance with the same parameters as the current object.

        """
        new_instance = Instance(ig.Graph(n=len(self.agent_graph.vs)), 
                        ig.Graph(), 
                        self.config_start, 
                        self.config_end, self.heuristic)
        new_instance.agent_graph = self.agent_graph.copy()
        new_instance.comm_graph = self.comm_graph.copy()
        new_instance.deterministic_graph = self.deterministic_graph.copy()
        new_instance.seed = self.seed
        return new_instance
        
    
    def edge_present(self, node_1, node_2):
        """
        Used to see if there are an edge in the deterministic graph between
        the two nodes.

        Parameters
        ----------
        node_1 : int
            The index of the first node.
        node_2 : int
            The index of the second node.

        Returns
        -------
        bool
            True if the edge (node1, node2) is present or if node1 == node2,
            False otherwise.

        """
        present = self.deterministic_graph.get_eid(node_1, node_2, error=False)
        return node_1 == node_2 or not present == -1
        
    
    def make_proba(self, graphe):
        """
        Generate the attribute "proba" in the graph if it is not already exist. 

        Parameters
        ----------
        graphe : Graph
            The graph to inspect.

        Returns
        -------
        None.

        """
        if not "proba" in graphe.es.attribute_names():
            graphe.es["proba"] = [0.0] * len(graphe.es)
        

    def generate_deterministic(self, graphe):
        """
        Generate the deterministic graph which represent the real graph 
        where the agents will evolved in.

        Parameters
        ----------
        graphe : Graph
            A graph with a "proba" attribute on his edge.

        Returns
        -------
        graphe : Graph
            The final deterministic graph.

        """
        edge_to_delete = []
        for edge in graphe.es:
            if r.random() < edge["proba"]:
                edge_to_delete.append(edge)
        graphe.delete_edges(edge_to_delete)
        return graphe
    

    def init_distance(self):
        """
        Initialize the matrix of distance between nodes in the agent_graph.
        The default value of this distance is None.

        Returns
        -------
        None.

        """
        self.matrix_distance = [[None for i in range(len(self.agent_graph.vs))] 
                                for j in range(len(self.agent_graph.vs))]
  
    def clear_distance(self):
        """
        Clear all the distances in the matrix distance back to None.

        Returns
        -------
        None.

        """
        self.init_distance()
    

    def get_distance(self, node_1, node_2):
        """
        Get the distance between two nodes in the agent_graph using the 
        attribute heuristic.

        Parameters
        ----------
        node_1 : int
            The index of the first node.
        node_2 : int
            The index of the second node.

        Returns
        -------
        float
            The distance between the two nodes.

        """
        #To compute only the upper part of the matrix
        if node_1 > node_2:
            tmp = node_1
            node_1 = node_2
            node_2 = tmp
        if isinstance(self.matrix_distance[node_1][node_2], type(None)):
            if self.heuristic == "euclide":
                self.matrix_distance[node_1][node_2] = self.euclidean_distance(
                    node_1, node_2)
            if self.heuristic == "manhattan":
                self.matrix_distance[node_1][node_2] = self.manhattan_distance(
                    node_1, node_2)
            if self.heuristic == "astar":
                if a_star(self, node_1, node_2) == 0:
                    return m.inf
        return self.matrix_distance[node_1][node_2]
    

    def add_distance(self, node_1, node_2, cost):
        """
        Add a distance cost from node_1 to node_2 in the matrix distance.
        Used by astar to fill the matrix.

        Parameters
        ----------
        node_1 : int
            The index of the first node.
        node_2 : int
            The index of the second node.
        cost : float
            The distance between node_1 and node_2.

        Returns
        -------
        None.

        """
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
        """
        Return a tuple with the coordinate of the node.

        Parameters
        ----------
        node : int
            The index of the node.

        Returns
        -------
        x : int
            The x coordinate of node.
        y : int
            The y coordinate of node.

        """
        x = self.agent_graph.vs[node]["x_coord"]
        y = self.agent_graph.vs[node]["y_coord"]
        return x, y

    """
    Utilitary functions
    """
    

    def print_grid(self):
        """
        Print a grid graph. Works only if the deterministic graph was build 
        with a Grid object.

        Returns
        -------
        None.

        """
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
    

    def euclidean_distance(self, node_1, node_2):
        """
        The euclidean distance between two nodes in the agent_graph.

        Parameters
        ----------
        node_1 : int
            The index of the first node.
        node_2 : int
            The index of the second node.

        Returns
        -------
        float
            The euclidean distance between the two nodes.

        """
        x_a1, y_a1 = self.get_position(node_1)
        x_a2, y_a2 = self.get_position(node_2)
        
        return m.sqrt((x_a1 - x_a2)*(x_a1 - x_a2) + (y_a1 - y_a2)*(y_a1 - y_a2))


    def manhattan_distance(self, node_1, node_2):
        """
        The Manhattan distance between the two nodes in the agent_graph.

        Parameters
        ----------
        node_1 : int
            The index of the first node.
        node_2 : int
            The index of the second node.

        Returns
        -------
        float
            The Manhattan distance between the two nodes.

        """
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