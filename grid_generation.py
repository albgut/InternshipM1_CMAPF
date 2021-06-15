#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 10 08:40:48 2021

@author: algutier
"""

import igraph as ig
import random as r

class Grid:
    
    def __init__(self, h, w):
        self.h = h
        self.w = w
        self.graphe_m = ig.Graph()
        self.graphe_m.add_vertices(h * w)
        for i in range(h):
            for j in range(w):
                current_node_id = self.get_num_vertice(i, j)
                self.graphe_m.vs[current_node_id]["x_coord"] = j
                self.graphe_m.vs[current_node_id]["y_coord"] = i
                if not i == 0:
                    self.generate_edge(i - 1, j, current_node_id)
                    if not j == 0:
                        self.generate_edge(i - 1, j - 1, current_node_id)
                    if not j == w - 1:
                        self.generate_edge(i - 1, j + 1, current_node_id)
                if not j == w - 1:
                    self.generate_edge(i, j + 1, current_node_id)
                    
    def generate_edge(self, i, j, source):
        target = self.get_num_vertice(i, j)
        self.graphe_m.add_edges([(target, source)])
        edge_id = self.graphe_m.get_eid(target, source)
        self.generate_random(edge_id)
        
    def generate_random(self, edge_id):
        if r.random() < 0.5:
            rand = r.randint(1, 9)
            self.graphe_m.es[edge_id]["proba"] = round(rand * 0.1, 1)
        else:
            self.graphe_m.es[edge_id]["proba"] = 0.0
                    
    def print_grid(self):
        print(self.graphe_m)
        print(self.graphe_m.es["proba"])
                    
    def get_num_vertice(self, i, j):
        return i * self.w + j
                    

if __name__ == "__main__":
    g = Grid(3, 3)
    g.print_grid()