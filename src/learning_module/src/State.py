# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 16:19:53 2015

@author: Prashanth
"""
import copy
import random as r
from StateAction import StateAction

class State:
    def __init__(self, label, configuration):
        self.configuration = configuration
        self.actionable_blocks = []
        self.label = label
        self.actions = []
        self.dist = 0.0
        """END"""
    def createStateActions(self):
        self.actionable_blocks = list(self.configuration)        
        non_actionable_blocks = [] 
        actions = []
        # Find the address of all actionable blocks
        for block in self.configuration:
            if block > -1:
                non_actionable_blocks.append(block)
        for nb in non_actionable_blocks:
            self.actionable_blocks[nb] = -2
        for ablock, ablock_status in enumerate(self.actionable_blocks):
            if(ablock_status!= -2):
                if(ablock_status >-1):
                    actions.append(StateAction(ablock))
                for dblock, dblock_status in enumerate(self.actionable_blocks):
                    if dblock_status!=-2 and dblock!=ablock:
                        actions.append(StateAction(ablock, dblock))
                
        self.actions = actions
        """END"""
    def chooseStateAction(self, probabilities):
        rand_num = r.random()
        sum_val = 0.0
        for action in self.actions:
            sum_val+=probabilities[action.getNextStateAddr()]
            if rand_num < sum_val:
                return action
        print "#################No Action Found#################"
        return None 
        """END"""
    def chooseBestStateAction(self, qlist):
        temp_list = []
        for qval in qlist:
            if qval != 0:
                temp_list.append(qval)
        max_qval = max(temp_list)
        addr = qlist.index(max_qval)
        for action in self.actions:
            if action.getNextStateAddr() == addr:
                return action
        """END"""
    def getLabel(self):
        return self.label
    def getActions(self):
        return self.actions
    def getConfiguration(self):
        return self.configuration