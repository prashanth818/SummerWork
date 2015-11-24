# -*- coding: utf-8 -*-
"""
Created on Mon Jul 20 14:06:48 2015

@author: Prashanth
"""

"""
Node is either an attribute or a leaf.
A node would contain its own subset. This it receives from the parent node and
branch it came from.
A node contains its children nodes.
A node contains its own set of information.
It needs to be seperated
"""
from copy import deepcopy
from scipy import stats
import numpy as np
import random
import operator
class Node:
    def __init__(self, attribute, training_set):
        self.attribute = attribute
        self.children_nodes = []
        self.children_attr = []
        self.child_link = []
        self.rules = []
        self.addr = None
        self.parent_addr = None
        self.training_set = training_set
        self.leaf = False
        self.bound = 0.0
    def isLeafNode(self):
        if self.attribute == None:
            return True
        elif self.leaf == True:
            return True
        else:
            return False
    def setLeafNode(self):
        self.leaf = True
    def setChildrenAttributes(self, children_attr):
        self.children_attr = children_attr
    def getChildrenAttributes(self):
        return self.children_attr
    def setRules(self, rules):
        self.rules = rules
    def getRules(self):
        return self.rules
    def getAttribute(self):
        return self.attribute
    def getBound(self):
        return self.bound
    def setBound(self, bound):
        self.bound = bound
    def addChildNode(self, child):
        self.children_nodes.append(child)
    def addChildLink(self, addr):
        self.child_link.append(addr)
    def getChildLink(self):
        return self.child_link
    def getTrainingSet(self):
        return self.training_set
    def getChildNodes(self):
        return self.children_nodes
    def setAddr(self, addr):
        self.addr = addr
    def getAddr(self):
        return self.addr
    def setParentAddr(self, parent_addr):
        self.parent_addr = parent_addr
    def getParentAddr(self):
        return self.parent_addr

class Attribute:
    def __init__(self, name, index, values):
        self.name = name
        self.index = index
        self.values = values
    def getName(self):
        return self.name
    def getIndex(self):
        return self.index
    def getValues(self):
        return self.values

class DecisionTree:
    def __init__(self, attributes, training_sample, test_sample = None):
        self.parent_node = None
        self.attributes = attributes
        self.training_sample = training_sample
        self.test_sample = test_sample
        self.rules = []
        self.tree = []
    def getRules(self):
        self.createDecisionTree()
        rules = []
        for rule in self.rules:
            average = 0.0
            for q_val in rule[-1]:
                average += q_val[-1]
            average /= len(rule[-1])
            rules.append([rule[0], average])
        return rules
    def findChildrenNode(self, current_node, attributes, subset, parent_length):
        child_node = self.findNextBestNode(attributes, subset[1], parent_length)
        if child_node != None:
            new_rule = current_node.getRules()+[(current_node.getAttribute().getName()+subset[0]+")")]
            child_node.setRules(new_rule)
            position = self.findPositionOfAttribute(attributes, child_node)
            child_attrs = attributes[:position]+attributes[position+1:]
            child_node.setChildrenAttributes(child_attrs)
        else:
            #print len(attributes)
            child_node = self.addLeafNode(current_node)
        return child_node
    def isBoundNode(self, parent_node, child_nodes):
        child_bound = 0.0
        for node in child_nodes:
            child_bound += node.getBound()
        child_bound = child_bound/len(child_nodes)
        if child_bound > parent_node.getBound():
            return True
        else:
            return False
    def addLeafNode(self, current_node):
        last_attr = current_node.getTrainingSet()[0][current_node.getAttribute().getIndex()]
        new_rule = current_node.getRules()+[(current_node.getAttribute().getName()+ last_attr +")")]
        leaf_node = Node(None, current_node.getTrainingSet())
        leaf_node.setRules(new_rule)
        return leaf_node

    def createDecisionTree(self, current_node = None):
        if current_node == None:
            current_node = self.findNextBestNode(self.attributes, self.training_sample, len(self.training_sample))
            position = self.findPositionOfAttribute(self.attributes, current_node)
            child_attrs = self.attributes[:position]+self.attributes[position+1:]
            current_node.setChildrenAttributes(child_attrs)
            current_node.setAddr(len(self.tree))
            self.tree.append(current_node)
        if not current_node.getChildrenAttributes():
            return
        subsets = self.createSubsets(current_node)
        other_attr = deepcopy(current_node.getChildrenAttributes())
        for subset in subsets:
            child_node = self.findChildrenNode(current_node, other_attr, subset, len(current_node.getTrainingSet()))
            child_node.setParentAddr(current_node.getAddr())
            current_node.addChildNode(child_node)
            current_node.addChildLink(len(self.tree))
            self.tree.append(child_node)
        child_nodes = deepcopy(current_node.getChildNodes())
        for node in child_nodes:
            self.createDecisionTree(node)

    def pruneDecisionTree(self):
        for node in self.tree:
            if node != None:
                if node.isLeafNode():
                    parent_node = self.tree[node.getParentNode()]
                    leaf_nodes = []
                    for addr in parent_node.getChildLink():
                        if self.tree[addr].isLeafNode():
                            leaf_nodes.append(self.tree[addr])
                        else:
                            leaf_nodes = []
                    if len(leaf_nodes)>0:
                        is_bound = self.isBoundNode(parent_node, leaf_nodes)
                        if is_bound == True:
                            for node in leaf_nodes:
                                self.tree[node.getAddr()] = None
                            parent_node.setLeafNode()


    def averageQValue(self, trainingSet):
        qvals = []
        for t in trainingSet:
            qvals.append(t[-1])
        average = np.mean(qvals)
        return average

    def findPositionOfAttribute(self, attributes, node):
        i = 0
        for attr in attributes:
            if node.getAttribute().getName() == attr.getName():
                return i
            i+=1
        print "Not Found"
        return None

    def prioritizeSubsets(self, subsets):
        subsets.sort(key=lambda x: x[2])
        return subsets

    def getParentNode(self):
        return self.parent_node

    def calcVariance(self, qlist):
        average = float(sum(qlist))/float(len(qlist))
        variance = 0.0
        for q_val in qlist:
            variance += (average-q_val)**2
        return variance

    def createSubsets(self, current_node):
        temp_subsets = []
        subsets = []
        for value in current_node.getAttribute().getValues():
            temp_subsets.append([value, [], 0.0])
        for example in current_node.getTrainingSet():
            for value in temp_subsets:
                if len(example) > 4:
                    if example[current_node.getAttribute().getIndex()] == value[0]:
                        value[1].append(example)
        for value in temp_subsets:
            if len(value[1]) > 0:
                subsets.append(value)
        for value in subsets:
            qlist = []
            for example in value[1]:
                qlist.append(example[-1])
            variance = self.calcVariance(qlist)
            value[2] = variance
        return subsets

    def calculateBound(self, qlist):
        error_list = []
        mean = np.mean(qlist)
        for q_val in qlist:
            error_list.append(mean-q_val)
        mean = np.mean(error_list)
        sigma = np.std(error_list)
        lower, upper = stats.norm.interval(0.68, loc=mean, scale=sigma)
        return upper

    def findNextBestNode(self, attributes, training_sample, parent_length):
        current_best = None
        max_gain = 0.0
        qlist = []
        for sample in training_sample:
		qlist.append(sample[-1])
        original_var = self.calcVariance(qlist)
        for attr in attributes:
            temp_node = Node(attr, training_sample)
            temp_subsets = self.createSubsets(temp_node)
            if len(attributes) > 1:
                var_sum = 0.0
                for each in temp_subsets:
                    var_sum += each[-1]
                info_gain = original_var - var_sum
                if max_gain < info_gain:
                    current_best = temp_node
                    max_gain = info_gain
            else:
                current_best = temp_node
                max_gain = 0.0
                break;
        if current_best == None:
            rand_attr = random.choice(attributes)
            current_best = Node(rand_attr, training_sample)
        bound = self.calculateBound(qlist)*(float(len(qlist))/float(parent_length))
        current_best.setBound(bound)
        return current_best
