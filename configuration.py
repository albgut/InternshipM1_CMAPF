#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun 12 15:51:47 2021

@author: algutier
"""

class Configuration:
    """
    The representation of a configuration:
        - l_config : Array<int>
            The list of the index of the node for each agent.
        - nb_agent : int
            The number of agent in the cnofiguration.
    """
    
    def __init__(self, l_config):
        """
        Create a new configuration from a list of index of nodes.

        Parameters
        ----------
        l_config : Array<int>
            The list of the index of the node for each agent.

        Returns
        -------
        None.

        """
        self.l_config = l_config
        self.nb_agent = len(l_config)
      

    def get_agent_pos(self, num_agent):
        """
        Get the index of the node of the agent in the configuration.

        Parameters
        ----------
        num_agent : int
            The index of the agent.

        Returns
        -------
        int
            The index of the node of the agent num_agent.

        """
        return self.l_config[num_agent]
    

    def same(self, config):
        """
        Verify if the two configuration have the same nodes index for 
        each agent.

        Parameters
        ----------
        config : Configuration
            The configuration to compare.

        Returns
        -------
        bool
            True if the configuration are the same, False otherwise.

        """
        if self.nb_agent != config.nb_agent:
            return False
        else:
            for c1, c2 in zip(self.l_config, config.l_config):
                if c1 != c2:
                    return False
        return True
    
    def change_pos(self, num_agent, new_agent_position):
        """
        Change the position of an agent in the configuration.

        Parameters
        ----------
        num_agent : int
            The index of the current agent.
        new_agent_position : int
            The index of the node of the new position.

        Returns
        -------
        None.

        """
        self.l_config[num_agent] = new_agent_position
    

    def add_agent(self, agent_position):
        """
        Add a position for a new agent at the end of the configuration.

        Parameters
        ----------
        agent_position : int
            The index of the node of this agent.

        Returns
        -------
        None.

        """
        self.nb_agent += 1
        self.l_config.append(agent_position)
    

    def copy(self):
        """
        Returns a copy of the configuration.

        Returns
        -------
        Configuration
            The copy of the configuration.

        """
        return Configuration(self.l_config.copy())
    

    def is_empty(self):
        """
        Verify if the configuration is empty.

        Returns
        -------
        bool
            True if the configuration is empty (without agent), 
            False otherwise.

        """
        return self.nb_agent == 0
    
    def __str__(self):
        """
        The toString method for print the configuration.

        Returns
        -------
        string
            The string representation of the configuration.

        """
        return self.l_config.__str__()
    
    def __eq__(self, config):
        """
        The equality method to verify the equality of two configurations.

        Parameters
        ----------
        config : Configuration
            The configuration to compare.

        Returns
        -------
        bool
            True if the configurations are the same, False otherwise.

        """
        return self.same(config)
    
    def __lt__(self, config):
        """
        The less than method to compare two configurations.

        Parameters
        ----------
        config : Configuration
            The configuration to compare.

        Returns
        -------
        bool
            True if the current object is less than config, False otherwise.

        """
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
    
    def __hash__(self):
        return hash(tuple(self.l_config))
    
if __name__ == "__main__":
    c = Configuration([1, 2, 3, 4])
    print(c)
    print(c.get_agent_pos(2))
    print(c.nb_agent)
    print(c.same(Configuration([1,2,3,5])))