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
import random
import operator
class Node:
    def __init__(self, attribute, training_set):
        self.attribute = attribute
        self.children_nodes = []
        self.children_attr = []
        self.rules = []
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
    def getTrainingSet(self):
        return self.training_set
    def getChildNodes(self):
        return self.children_nodes

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

    def findChildrenNode(self, current_node, attributes, subset):
        child_node = self.findNextBestNode(attributes, subset[1])
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

    def getBoundNode(self, child_nodes):
        print child_nodes
        child_nodes = sorted(child_nodes, key=operator.attrgetter('bound'), reverse = True)
        return child_nodes[0]

    def addLeafNode(self, current_node):
        last_attr = current_node.getTrainingSet()[0][current_node.getAttribute().getIndex()]
        new_rule = current_node.getRules()+[(current_node.getAttribute().getName()+ last_attr +")")]
        leaf_node = Node(None, current_node.getTrainingSet())
        leaf_node.setRules(new_rule)
        return leaf_node

    def createDecisionTree(self, current_node = None):

        if current_node == None:
            current_node = self.findNextBestNode(self.attributes, self.training_sample)
            position = self.findPositionOfAttribute(self.attributes, current_node)
            child_attrs = self.attributes[:position]+self.attributes[position+1:]
            current_node.setChildrenAttributes(child_attrs)
        if not current_node.getChildrenAttributes():
            last_val = current_node.getTrainingSet()[0][current_node.getAttribute().getIndex()]
            new_rule = current_node.getRules()+[(current_node.getAttribute().getName()+last_val+")")]
            self.rules.append([new_rule, current_node.getTrainingSet()])
            return self.rules
        subsets = self.createSubsets(current_node)
        other_attr = deepcopy(current_node.getChildrenAttributes())
        for subset in subsets:
            child_node = self.findChildrenNode(current_node, other_attr, subset)
            current_node.addChildNode(child_node)
        child_nodes = deepcopy(current_node.getChildNodes())
        best_node = self.getBoundNode(child_nodes)
        self.rules.append([best_node.getRules(),best_node.getTrainingSet()])
        if best_node != None and len(child_nodes)>1:
            child_nodes.remove(best_node)
        for node in child_nodes:
            self.createDecisionTree(node)

    def averageQValue(self, trainingSet):
        qvals = []
        for t in trainingSet:
            qvals.append(t[-1])
        average = sum(qvals)/len(qvals)
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
        if len(qlist) != 0:
            average = float(sum(qlist))/float(len(qlist))
            variance = 0.0
            for qval in qlist:
                variance += (average - qval)**2
            return variance
        else:
            return None

    def createSubsets(self, current_node):
        temp_subsets = []
        subsets = []
        for value in current_node.getAttribute().getValues():
            temp_subsets.append([value, [], 0.0])
        for example in current_node.getTrainingSet():
            for value in temp_subsets:
                if len(example) > 4:
                    # print example
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

    def findNextBestNode(self, attributes, training_sample):
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
        current_best.setBound(max_gain)
        return current_best
