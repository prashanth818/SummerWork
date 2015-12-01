#!/usr/bin/env python
"""
Created on Wed Jul 15 12:16:58 2015

@author: Prashanth
"""

"""This is a file that is purely for testing the relational reinforcement section."""

#import rospy
#import re
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

class LearningModule:
    def __init__(self):
        self.mdp_list = []
        self.success_config = []
        self.decision_tree = None
        self.StateActionPairs= []
        # initialise
        self.mdp_list.append([])

    def initialiseAttributes(self):
        binary_values = ("true", "false")
        attributes = ["on(b0, table)", "on(b0, b1)", "on(b0,b2)",
        "on(b1, table)", "on(b1, b0)", "on(b1, b2)",
        "on(b2, table)", "on(b2, b0)", "on(b2,b1)",
        "has_shape(b0, prism)", "has_shape(b1, prism)", "has_shape(b2, prism)",
        "has_shape(b0, cube)", "has_shape(b1, cube)", "has_shape(b2, cube)",
        "has_shape(b0, cuboid)", "has_shape(b1, cuboid)", "has_shape(b2, cuboid)",
        "has_colour(b0, red)", "has_colour(b1, red)", "has_colour(b2, red)",
        "has_colour(b0, blue)", "has_colour(b1, blue)", "has_colour(b2, blue)",
        "has_colour(b0, green)", "has_colour(b1, green)", "has_colour(b2, green)",
        "has_size(b0, small)", "has_size(b1, small)", "has_size(b2, small)",
        "has_size(b0, medium)", "has_size(b1, medium)", "has_size(b2, medium)",
        "has_size(b0, large)", "has_size(b1, large)", "has_size(b2, large)",
        "move(b0, table)", "move(b0, b1)", "move(b0, b2)",
        "move(b1, table)", "move(b1, b0)", "move(b1, b2)",
        "move(b2, table)", "move(b2, b0)", "move(b2, b1)"]
        attribute_dict = []
        for attribute in attributes:
            attribute_dict.append((attribute,binary_values))
        attribute_dict = OrderedDict(attribute_dict)
        index = 0
        names = attribute_dict.keys()
        values = attribute_dict.values()
        attributes = []
        for name, vals in zip(names, values):
            attributes.append(Attribute(name, index, vals))
            index += 1
        return attributes

    def initialise_mdp(self, blocks):
        start_config = [-1,-1,-1]
        startingState = State(0, blocks, start_config)
        self.initialise_lists()
        self.success_config[-1].append(startingState)
        label = len(self.mdp_list[-1])
        mdp = MDP(label, blocks)
        mdp.statelist.append(startingState)
        mdp.initMDP(startingState)
        self.mdp_list[-1].append(mdp)

    def new_layer(self):
        try:
            self.mdp_list.append([])
            return True
        except:
            return False

    def initialise_lists(self):
        self.success_config.append([])

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

    def errorHandle(self, error_config, success_config, attributes):
        success_states = []
        for config  in success_config:
            success_states.append(self.findState(config, self.mdp_list[-1][-1]))
        error_state = self.findState(error_config, self.mdp_list[-1][-1])
        self.mdp_list[-1][-1].simulation(error_state, self.success_config[-1], attributes)

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

    def generateRules(self):
        reduced_mdp_list = []
        attributes = []
        self.mdp_list[-1] = [self.combineIdenticalMDPs(self.mdp_list[-1])]
        self.writeToList(self.mdp_list[-1][-1])
        training_set = self.StateActionPairs

        attr_shape = ("cube", "prism", "cuboid")
        attr_colour = ("red", "blue", "green")
        attr_size = ("small","medium","large")
        attribute_dict = [("has_shape(A,", attr_shape), ("has_colour(A,", attr_colour), ("has_size(A,", attr_size),
                            ("has_shape(D,", attr_shape), ("has_colour(D,",attr_colour), ("has_size(D,", attr_size)]
        attribute_dict = OrderedDict(attribute_dict)
        index = 0
        names = attribute_dict.keys()
        values = attribute_dict.values()
        for name, vals in zip(names, values):
            attributes.append(Attribute(name, index, vals))
            index += 1
        self.decision_tree = DecisionTree(attributes, training_set)
        rules = self.decision_tree.getRules()
        for rule in rules:
            print rule
        # rules = self.selectRules(rules)

    def selectRules(self, rules):
        """ Select the best rules """
        """ Think about doing it using SVM"""
        print rules
        for rule in rules:
            print rule
        print "\n"
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
        return valid_rules

    def reduceMDP(self,errorconfig, stack_config, start_config, blocks):
        mdp_list = []
        attributes = self.initialiseAttributes()
        for i in range(0, len(errorconfig)):
            mdp_list.append(MDP(i, blocks))
            startingState = State(0, start_config)
            mdp_list[i].statelist.append(startingState)
            mdp_list[i].initMDP(startingState)
            errorstate = self.findState(errorconfig[i], mdp_list[i])
            stackstate = []
            for j in range(0,len(stack_config)):
                stackstate.append(self.findState(stack_config[j], mdp_list[i]))
            mdp_list[i].simulation(errorstate, stackstate, attributes)
            mdp_list[i].updateDistanceMatrix(errorstate)

        reduced_mdp = self.combineIdenticalMDPs(mdp_list)
        return reduced_mdp

def shapeScenario(learning_module, attributes):
    blocks_shape = []
    block0 = Block(0, "prism", "blue", "small")
    block1 = Block(1, "cube", "blue", "medium")
    block2 = Block(2, "cuboid", "red", "small")
    blocks_shape.append(block0)
    blocks_shape.append(block1)
    blocks_shape.append(block2)
    errors_shape = [[-1,0,-1],[-1,-1,0],[1,-1,0],[2,0,-1]]
    success_shape = [[-1,-1,-1],[1,2,-1],[2,-1,1],[-1,2,-1],[-1,-1,1]]
    for error_config in errors_shape:
        learning_module.initialise_mdp(blocks_shape)
        learning_module.errorHandle(error_config, success_shape, attributes)
    print "i get here"
    for mdp in learning_module.mdp_list[-1]:
        print len(mdp.getStateList())
    print "\n"
    learning_module.generateRules()
    learning_module.new_layer()
    print "First set of blocks done"
    blocks_shape = []
    block0 = Block(0, "prism", "red", "medium")
    block1 = Block(1, "cube", "blue", "medium")
    block2 = Block(2, "cuboid", "green", "medium")
    blocks_shape.append(block0)
    blocks_shape.append(block1)
    blocks_shape.append(block2)
    for error_config in errors_shape:
        learning_module.initialise_mdp(blocks_shape)
        learning_module.errorHandle(error_config, success_shape, attributes)
    print "i get here also"
    for mdp in learning_module.mdp_list[-1]:
        print len(mdp.getStateList())
    print "\n"
    learning_module.generateRules()
    learning_module.new_layer()

    blocks_shape = []
    block0 = Block(0, "prism", "green", "small")
    block1 = Block(1, "cube", "red", "medium")
    block2 = Block(2, "cuboid", "green", "medium")
    blocks_shape.append(block0)
    blocks_shape.append(block1)
    blocks_shape.append(block2)
    for error_config in errors_shape:
        learning_module.initialise_mdp(blocks_shape)
        learning_module.errorHandle(error_config, success_shape, attributes)
    for mdp in learning_module.mdp_list[-1]:
        print len(mdp.getStateList())
    print "\n"
    return learning_module.generateRules()

def scenarioGenerator(test, learning_module, attributes):
    switcher = {
    "shape":shapeScenario(learning_module, attributes),
    "colour":colourScenario(learning_module),
    "size":sizeScenario(learning_module)
    }
    return switcher.get(test)

def main():
    learning_module = LearningModule()
    attributes = learning_module.initialiseAttributes()
    rules = scenarioGenerator("shape", learning_module, attributes)
    for rule in rules:
        print rule

if __name__ == '__main__':
    main()
