#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from configuration import *


class Closed_tree:
    def __init__(self):
        self.root = Closed_node(-1)
      
    def is_in(self, configuration):
        if configuration.nb_agent == 0:
            return True
        i = 0
        current_node = self.root
        while i < configuration.nb_agent:
            next_node = current_node.find_child_by_value(
                configuration.get_agent_pos(i))
            if next_node == None:
                return False
            else:
                i += 1
                current_node = next_node
        return True
    
    def clear(self):
        self.root = Closed_node(-1)
        
    def add_configuration(self, configuration):
        if not configuration.nb_agent == 0:
            i = 0
            current_node = self.root
            while i < configuration.nb_agent:
                next_node = current_node.find_child_by_value(
                    configuration.get_agent_pos(i))
                if next_node == None:
                    current_node = current_node.add_child(
                        configuration.get_agent_pos(i))
                else:
                    current_node = next_node
                i += 1
                
    def print_tree(self):
        self.root.print_node("")
        
class Closed_node:
    def __init__(self, node_value):
        self.value = node_value
        self.children = dict()
        
    def find_child_by_value(self, value):
        return self.children.get(value)
    
    def add_child(self, node_value):
        new_node = Closed_node(node_value)
        self.children[node_value] = new_node
        return new_node
    
    def print_node(self, indent):
        print(indent + str(self.value))
        for key, child in self.children.items():
            child.print_node(indent + "\t")
        

if __name__ == "__main__":
    ct = Closed_tree()
    ct.add_configuration(Configuration([1, 5]))
    ct.add_configuration(Configuration([5, 1]))
    ct.add_configuration(Configuration([1, 2]))
    ct.print_tree()
    print(ct.is_in(Configuration([1, 5])))
    print(ct.is_in(Configuration([1, 0])))
    ct.clear()
    print("tree cleared")
    ct.print_tree()