# -*- coding: utf-8 -*-
"""
Created on Mon Jul 20 13:05:29 2015

@author: Prashanth
"""

import math
import copy
from State import State
import random as r

class MDP:
    def __init__(self, label, blocks):
        self.label = label
        self.blocks = blocks
        self.qmat = []
        self.rewardmat = []
        self.probmat = []
        self.statelist = []
        self.distance_matrix = []
        self.errorstate = None
        print "mdpppp"
    def getLabel(self):
		return self.label
    def getStateList(self):
        return self.statelist
    def getQMatrix(self):
        return self.qmat
    def getDistanceMatrix(self):
        return self.distance_matrix
    def getBlocks(self):
        return self.blocks
    def setQMatrix(self, qmat):
        self.qmat = qmat
    def getErrorState(self):
        return self.errorstate
    def findNextState(self, currentstate, action):
        newconfig = list(currentstate.getConfiguration())
        for i in range(0, len(newconfig)):
            if i == action.getActionableBlock():
                if action.getDestinationBlock() == None:
                    newconfig[i] = -1
                else:
                    newconfig[i] = action.getDestinationBlock()
        flag = False
        for state in self.statelist:
            for c, nc in zip(state.getConfiguration(), newconfig):
                if c != nc:
                    flag = False
                    break
                flag = True
            if flag == True:
                action.setNextStateAddr(state.getLabel())
                action.setVisited(True)
                return state
        if flag == False:
            newstate = State(len(self.statelist), newconfig)
            action.setNextStateAddr(newstate.getLabel())
            action.setVisited(True)
            self.statelist.append(newstate)
            return newstate
        """END"""

    def initStateList(self, currentstate):
        if not currentstate.getActions():
            currentstate.createStateActions()
        for action in currentstate.getActions():
            if action.isVisited() == False:
                nextstate = self.findNextState(currentstate, action)
                self.initStateList(nextstate)
        return
        """END"""

    def initMDP(self, initstate):
        self.initStateList(initstate)
        self.errorstate = initstate
        self.qmat = [[0.0 for i in range(0,len(self.statelist))] for i in range(0,len(self.statelist))]
        self.probmat = [[0.0 for i in range(0,len(self.statelist))] for i in range(0,len(self.statelist))]
        self.distance_matrix = [[0.0 for i in range(0,len(self.statelist))] for i in range(0,len(self.statelist))]
        for state in self.statelist:
            for action in state.getActions():
                if(action.isVisited()):
                    action.setVisited(False)
                self.probmat[state.getLabel()][action.getNextStateAddr()] = 1.0/len(state.getActions())
                self.distance_matrix[state.getLabel()][action.getNextStateAddr()] = 1
        self.rewardmat = [-0.25 for i in range(0,len(self.statelist))]
        """END"""

    def simulation(self, errorstate, stackstate):
        #self.rewardmat[errorstate.getLabel()][error_action_chosen.getNextStateAddr()] = 5
        #self.reward_matrix = reward_matrix
        print "Simulating"
        for state in stackstate:
            self.rewardmat[state.getLabel()] = -2.2
            print "stack state"
        self.rewardmat[errorstate.getLabel()] = 5.5
        for i in range(0,2500):
            currentstate = r.choice(self.statelist)
            while(currentstate != self.statelist[errorstate.getLabel()]):
                action_chosen = currentstate.chooseStateAction(self.probmat[currentstate.getLabel()])
                nextstate = self.statelist[action_chosen.getNextStateAddr()]
                self.qLearning(currentstate,  action_chosen)
                self.updateProbabilityMatrix(self.probmat[currentstate.getLabel()], self.qmat[currentstate.getLabel()])
                currentstate = nextstate
            for state in self.statelist:
                for action in state.getActions():
                    action.setVisited(False)

        print "end of simulation"
        """END"""

    def onPolicyLearning(self, action_chosen):
        """
        If on policy learning is being done then this function must be continuously called.
        If no action is given, then it is a signal that an error has occured and an
        errorstate is returned.
        """
        print "on policy"
        self.qLearning(self.errorstate, action_chosen)
        self.updateProbabilityMatrix(self.probmat[self.errorstate.getLabel()], self.qmat[self.errorstate.getLabel()])
        nextstate = self.statelist[action_chosen.getNextStateAddr()]
        self.errorstate = nextstate
        """END"""

    def qLearning(self, currentstate, action_chosen):
        gamma = 0.75
        alpha = 1.0/(action_chosen.getNumVisits() + 2.0)
        reward = self.rewardmat[action_chosen.getNextStateAddr()]
        nextstate_maxqval = max(self.qmat[action_chosen.getNextStateAddr()])
        current_qval = self.qmat[currentstate.getLabel()][action_chosen.getNextStateAddr()]
        current_qval += alpha*(reward + gamma*(nextstate_maxqval) - current_qval)
        if(action_chosen.isVisited() == False):
            action_chosen.incrementNumVisits()
            action_chosen.setVisited(True)
        self.qmat[currentstate.getLabel()][action_chosen.getNextStateAddr()] = current_qval
        """
        Update the logical tree using the example.
        Use the leaf nodes to update all the state-action pairs
        """
        """END"""

    def updateProbabilityMatrix(self, probmat, qmat):
        sum_val = 0.0
        Temperature = 0.85
        for i,qval in enumerate(qmat):
            if probmat[i] > 0:
                sum_val += math.exp(qval/Temperature)
        for i, qval in enumerate(qmat):
            if probmat[i] > 0:
                prob = math.exp(qval/Temperature)
                prob /= sum_val
                probmat[i] = prob
        """END"""

    def updateDistanceMatrix(self, errorstate):
        for state in self.statelist:
            for action in state.getActions():
                currentstate = self.statelist[action.getNextStateAddr()]
                self.distance_matrix[state.getLabel()][action.getNextStateAddr()] += 1.0
                while currentstate != errorstate:
                    action_chosen = currentstate.chooseBestStateAction(self.qmat[currentstate.getLabel()])
                    self.distance_matrix[state.getLabel()][action.getNextStateAddr()] += 1.0
                    nextstate = self.statelist[action_chosen.getNextStateAddr()]
                    currentstate = nextstate
