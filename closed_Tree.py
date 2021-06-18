#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from configuration import *


class Closed_tree:
    """
    A tree of configuration.
    """
    
    def __init__(self):
        """
        Create a closed_tree object. The root is a closed_node object
        initialize to -1.

        Returns
        -------
        None.

        """
        self.root = Closed_node(-1)
      
    def is_in(self, configuration):
        """
        Verify if the configuration is already in the tree.

        Parameters
        ----------
        configuration : Configuration
            The configuration to rersearch.

        Returns
        -------
        bool
            Returns True if the configuration is in the tree, False otherwise.

        """
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
        """
        Clear the tree, delete all the configuration present in the tree.

        Returns
        -------
        None.

        """
        self.root = Closed_node(-1)
        
    def add_configuration(self, configuration):
        """
        Add a configuration in the tree.

        Parameters
        ----------
        configuration : Configuration
            The configuration to add.

        Returns
        -------
        None.

        """
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
        """
        Print the tree.

        Returns
        -------
        None.

        """
        self.root.print_node("")
        
class Closed_node:
    """
    A node in the Closed_tree
    """
    def __init__(self, node_value):
        """
        Create a new node with a value of node_value and without children.

        Parameters
        ----------
        node_value : int
            The value of the node to create.

        Returns
        -------
        None.

        """
        self.value = node_value
        self.children = dict()
        
    def find_child_by_value(self, value):
        """
        Find a child of the node which have this value.

        Parameters
        ----------
        value : int
            The value to research.

        Returns
        -------
        Closed_node
            The node with this value.

        """
        return self.children.get(value)
    
    def add_child(self, node_value):
        """
        Add a child to the node with this value, and returns this new node.

        Parameters
        ----------
        node_value : int
            The value of the new node.

        Returns
        -------
        new_node : Closed_node
            The new node created.

        """
        new_node = Closed_node(node_value)
        self.children[node_value] = new_node
        return new_node
    
    def print_node(self, indent):
        """
        Print the node value and the values of all his children.

        Parameters
        ----------
        indent : string
            An indentation for the lisibility.

        Returns
        -------
        None.

        """
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