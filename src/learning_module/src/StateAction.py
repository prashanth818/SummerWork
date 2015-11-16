# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 16:22:13 2015

@author: Prashanth
"""

class StateAction:
    def __init__(self, actionableBlock, destinationBlock = None):
        self.visited = False
        self.numvisits = 0.0
        self.actionableBlock = actionableBlock
        self.nextStateAddr = None
        self.destinationBlock = destinationBlock
        """END"""
    def isVisited(self):
        return self.visited
    def getNumVisits(self):
        return self.numvisits
    def incrementNumVisits(self):
        self.numvisits+=1
    def setVisited(self, visited):
        self.visited = visited
    def getActionableBlock(self):
        return self.actionableBlock
    def getDestinationBlock(self):
        return self.destinationBlock
    def setNextStateAddr(self, addr):
        self.nextStateAddr = addr
    def getNextStateAddr(self):
        return self.nextStateAddr
       