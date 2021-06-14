#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun 12 15:51:47 2021

@author: algutier
"""

"""
TODO
"""
class Configuration:
    
    def __init__(self, l_config):
        self.l_config = l_config
        self.nb_agent = len(l_config)
      
    """
    precond => 0 <= num_agent < self.nb_agent
    postcond => index of the node where is the agent num_agent
    """
    def get_agent_pos(self, num_agent):
        return self.l_config[num_agent]
    
    """
    Return true iff config and self have the same position for each agent
    """
    def same(self, config):
        if self.nb_agent != config.nb_agent:
            return False
        else:
            for c1, c2 in zip(self.l_config, config.l_config):
                if c1 != c2:
                    return False
        return True
    
    """
    Add a position for a new agent at the end of the configuration.
    Used to grow partial configuration
    """
    def add_agent(self, agent_position):
        self.nb_agent += 1
        self.l_config.append(agent_position)
    
    """
    Return a new object which is a copy of the configuration
    """
    def copy(self):
        return Configuration(self.l_config.copy())
    
    """
    return true iff the configuration is empty
    """
    def is_empty(self):
        return self.nb_agent == 0
    
    def __str__(self):
        return self.l_config.__str__()
    
    """
    HERE implementation to compare two config like tateo
    """
    def __eq__(self, config):
        return self.same(config)
    
    def __lt__(self, config):
        if self.nb_agent < config.nb_agent:
            return True
        if self.nb_agent > config.nb_agent:
            return False
        if self.nb_agent == config.nb_agent:
            for node_1, node_2 in zip(self.l_config, config.l_config):
                if node_1 < node_2:
                    return True
                if node_1 > node_2:
                    return False
        return False
    
if __name__ == "__main__":
    c = Configuration([1, 2, 3, 4])
    print(c)
    print(c.get_agent_pos(2))
    print(c.nb_agent)
    print(c.same(Configuration([1,2,3,5])))