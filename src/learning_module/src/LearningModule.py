#!/usr/bin/env python
"""
Created on Wed Jul 15 12:16:58 2015

@author: Prashanth
"""

"""This is a Service Node. Any requests will be responded with outputs."""

""" Onpolicy requests will respond with a boolean (success/ failure)"""
import rospy
import re
from MDP import MDP
from Block import Block
from State import State
from copy import deepcopy
from collections import OrderedDict
from scipy import stats
from scipy.cluster.vq import vq, kmeans, whiten
import numpy
import operator
from DecisionTree import *

from learning_module.srv import *
from learning_module.msg import *


from asp_module.msg import State as StateMsg
from asp_module.msg import Configuration as ConfigMsg
from asp_module.msg import Block as BlockMsg
class LearningModule:
    def __init__(self):
        self.mdp_list = []
        self.success_config = []
        self.decision_tree = None
        self.StateActionPairs= []

        src_error = rospy.Service('LMErrorHasOccured',LMErrorHasOccured, self.errorHandle)
        src_rules = rospy.Service('LMGenerateRules', LMGenerateRules, self.generateRules)
        srv_state = rospy.Service('LMInitialise', LMInitialise, self.initialise_mdp)
        srv_state = rospy.Service('LMNewBlocks', LMNewBlocks, self.newBlocks)
        srv_action = rospy.Service('LMStateActionTaken', LMStateActionTaken, self.onPolicyLearning)

        # initialise
        self.mdp_list.append([])


    def initialise_lists(self):
        self.success_config.append([])

    def initialise_mdp(self, state):
        try:
            blocks = []
            for prop in state.initial_state.block_properties:
                blocks.append(Block(prop.label, prop.shape, prop.colour, prop.size))
            start_config = state.initial_state.configuration.config
            startingState = State(0, start_config)
            self.initialise_lists()
            self.success_config[-1].append(startingState)
            label = len(self.mdp_list[-1])
            print ""
            print label
            print ""
            mdp = MDP(label, blocks)
            mdp.statelist.append(startingState)
            mdp.initMDP(startingState)
            self.mdp_list[-1].append(mdp)
            print "MDP initialised"
            return True
        except:
            return False

    def newBlocks(self, blockSet):
        try:
            # combine MDPS
            self.mdp_list[-1] = self.combineIdenticalMDPs(self.mdp_list[-1])
            # add combined MDPs state action pairs to the list!
            self.writeToList(self.mdp_list[-1][-1])
            # start new layer
            self.new_layer()
            return True
        except:
            return False

    def new_layer(self):
        try:
            self.mdp_list.append([])
            return True
        except:
            return False


    def combineIdenticalMDPs(self, mdp_list):
        print "combining"
        sum_distance = [[0.0 for i in range(0,len(mdp_list[0].getStateList()))] for j in range(0,len(mdp_list[0].getStateList()))]
        weighted_average = [[0.0 for i in range(0,len(mdp_list[0].getStateList()))] for j in range(0,len(mdp_list[0].getStateList()))]
        for mdp in mdp_list:
            for i, row in enumerate(mdp.getDistanceMatrix()):
                for j, distance in enumerate(row):
                    if distance > 0:
                        sum_distance[i][j] += 1/distance
        for mdp in mdp_list:
            for i, row in enumerate(mdp.getDistanceMatrix()):
                for j, distance in enumerate(row):
                    if distance > 0.0 and sum_distance[i][j] > 0.0:
                        weight = (1/distance)/(sum_distance[i][j])
                        weighted_average[i][j] += weight*mdp.getQMatrix()[i][j]

        newMDP = deepcopy(mdp_list[0])
        newMDP.setQMatrix(weighted_average)
        return newMDP

    def findState(self, config, mdp):
        for state in mdp.getStateList():
            if state.getConfiguration() == config:
                return state

    def errorHandle(self, action_chosen):
        # try:
        print "errr"
        action_chosen = action_chosen.action_chosen
        actionableBlock = int(re.findall('\d+$', action_chosen.actionableBlock)[0])
        destinationBlock = int(re.findall('\d+$', action_chosen.destinationBlock)[0])
        action_block = actionableBlock
        dest_block = destinationBlock
        action_chosen = None
        for action in self.mdp_list[-1][-1].getErrorState().getActions():
            if action.getActionableBlock() == action_block:
                if action.getDestinationBlock() == dest_block:
                    action_chosen = action

        self.mdp_list[-1][-1].onPolicyLearning(action_chosen)
        error_config = self.mdp_list[-1][-1].getErrorState()

        print self.success_config[-1]

        self.mdp_list[-1][-1].simulation(error_config, self.success_config[-1])
        return True
        # except:
        #     print "OMGMMMM"
        #     return False

    def onPolicyLearning(self, action):
        # try:
        """ This will be the callback function"""
        actionableBlock = int(re.findall('\d+$',action.action_chosen.actionableBlock)[0])
        if(re.findall('tab',action.action_chosen.destinationBlock)):
            print "###############TABLE################"
            destinationBlock = None
        else:
            destinationBlock = int(re.findall('\d+$',action.action_chosen.destinationBlock)[0])

        action_chosen = None

        for action in self.mdp_list[-1][-1].errorstate.actions:
            print action.actionableBlock
            print action.destinationBlock
            if(actionableBlock == action.actionableBlock) and (destinationBlock == action.destinationBlock):
                action_chosen = action


        self.mdp_list[-1][-1].onPolicyLearning(action_chosen)
        config = self.mdp_list[-1][-1].getErrorState()
        self.success_config[-1].append(config)
        return True

    def writeToList(self, mdp):
        blocks = mdp.getBlocks()
        for state in mdp.getStateList():
            for action in state.getActions():
                action_block = action.getActionableBlock()
                dest_block = action.getDestinationBlock()
                if dest_block == None:
                    example = (blocks[action_block].getShape(), blocks[action_block].getColour(), blocks[action_block].getSize(),
                               mdp.getQMatrix()[state.getLabel()][action.getNextStateAddr()])
                else:
                    example = (blocks[action_block].getShape(),
                               blocks[action_block].getColour(),blocks[action_block].getSize(), blocks[dest_block].getShape(),
                               blocks[dest_block].getColour(),blocks[dest_block].getSize(),
                               mdp.getQMatrix()[state.getLabel()][action.getNextStateAddr()])
                self.StateActionPairs.append(example)
        return

    def generateRules(self, randomCharacterBeingSentSomehow):

        print"generateRules"

        reduced_mdp_list = []
        attributes = []
        self.mdp_list[-1] = [self.combineIdenticalMDPs(self.mdp_list[-1])]
        self.writeToList(self.mdp_list[-1][-1])

        training_set = self.StateActionPairs

        attr_shape = ("cube", "prism", "cuboid")
        attr_colour = ("red", "blue", "green")
        attr_size = ("small","medium","large")
        attribute_dict = [("has_shape(A,", attr_shape), ("has_colour(A, ", attr_colour), ("has_size(A, ", attr_size),
                            ("has_shape(D, ", attr_shape), ("has_colour(D, ",attr_colour), ("has_size(D, ", attr_size)]
        attribute_dict = OrderedDict(attribute_dict)
        index = 0
        names = attribute_dict.keys()
        values = attribute_dict.values()
        for name, vals in zip(names, values):
            attributes.append(Attribute(name, index, vals))
            index += 1
        self.decision_tree = DecisionTree(attributes, training_set)
        rules = self.decision_tree.getRules()
        rules = self.selectRules(rules)
        print ""
        print rules
        return rules

    def selectRules(self, rules):
        """ Select the best rules """
        """ Think about doing it using SVM"""
        rules = sorted(rules, key=operator.itemgetter(-1))
        q_val = []
        for index, rule in enumerate(rules):
            q_val.append([index, rule[-1]])
        whitened = whiten(q_val)
        centroids,_ = kmeans(whitened, 3, thresh = 1,iter = 100)
        ids,_= vq(whitened, centroids)
        key = ids[-1]
        indices = []
        for index, keys in enumerate(ids):
            if key == keys:
                indices.append(index)
        valid_rules = []
        for index in indices:
            valid_rules.append(rules[index][0])
        return self.parseRules(valid_rules)

    def parseRules(self, rules):
        valid_rules = []
        for rule in rules:
            sentence = ""
            for segment in rule:
                sentence = sentence + segment + ", "
            sentence = sentence[:-2]
            valid_rules.append(sentence)
        return Rules(rule = valid_rules)

    def reduceMDP(self,errorconfig, stack_config, start_config, blocks):
        mdp_list = []
        for i in range(0, len(errorconfig)):
            mdp_list.append(MDP(i, blocks))
            startingState = State(0, start_config)
            mdp_list[i].statelist.append(startingState)
            mdp_list[i].initMDP(startingState)
            errorstate = self.findState(errorconfig[i], mdp_list[i])
            stackstate = []
            for j in range(0,len(stack_config)):
                stackstate.append(self.findState(stack_config[j], mdp_list[i]))
            mdp_list[i].simulation(errorstate, stackstate)
            mdp_list[i].updateDistanceMatrix(errorstate)

        reduced_mdp = self.combineIdenticalMDPs(mdp_list)
        return reduced_mdp



if __name__ == '__main__':
    rospy.init_node('learning', anonymous=True)
    LearningModule()

    print "Ready to learn!"
    rospy.spin()
