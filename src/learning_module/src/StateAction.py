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
        self.description = []
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

    def putDownQuery(self, blocks):
        """ b0, b1, b2"""
        """table, b0, b1, b2"""
        move_query = []
        for a_block in range(0, len(blocks)):
            if a_block == self.actionableBlock:
                if self.destinationBlock == None:
                    query = "true"#["move("+str(block_id)+",table)","true"]
                    move_query.append(query)
                else:
                    query = "false"#["move("+str(block_id)+",table)","false"]

                    move_query.append(query)
                for d_block in range(0, len(blocks)):
                    if a_block != d_block:
                        if d_block == self.destinationBlock:
                            query = "true"#["move("+str(block_id)+","+str(i)+")","true"]
                        else:
                            query = "false"#["move("+str(block_id)+","+str(i)+")","false"]
                        move_query.append(query)
            else:
                move_query.append("false")

                for d_block in range(0, len(blocks)):
                    if a_block != d_block:
                        query = "false"#["move("+str(block_id)+","+str(i)+")","false"]
                        move_query.append(query)
        return move_query
