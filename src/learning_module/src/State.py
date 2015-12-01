# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 16:19:53 2015

@author: Prashanth
"""
import copy
import random as r
from StateAction import StateAction

class State:
    def __init__(self, label, blocks, configuration):
        self.configuration = configuration
        self.blocks = blocks
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

    def onQuery(self):
        """ b0, b1, b2"""
        """table, b0, b1, b2"""
        on_query = []
        for block_id, config in enumerate(self.configuration):
            if config == -1:
                on_query.append("true")
            else:
                on_query.append("false")
            for i in range(0, len(self.blocks)):
                if block_id != i:
                    if config == i:
                        query = "true"
                    else:
                        query = "false"
                    on_query.append(query)
        return on_query

    def hasShapeQuery(self):
        """ b0, b1, b2"""
        """prism, cube, cuboid"""
        shapes = ["prism","cube","cuboid"]
        shape_query = []
        for b_id, block in enumerate(self.blocks):
            for shape in shapes:
                if shape == block.getShape():
                    query = "true"#["has_shape("+str(b_id)","+block.getShape()+")", "true"]
                else:
                    query = "false"#["has_shape("+str(b_id)","+block.getShape()+")", "false"]
                shape_query.append(query)
        return shape_query

    def hasColourQuery(self):
        """ b0, b1, b2"""
        """red, blue, green"""
        colours = ["red","blue","green"]
        colour_query = []
        for b_id, block in enumerate(self.blocks):
            for colour in colours:
                if colour == block.getColour():
                    query = "true"#["has_colour("+str(b_id)","+block.getColour()+")", "true"]
                else:
                    query = "false"#["has_colour("+str(b_id)","+block.getColour()+")", "false"]
                colour_query.append(query)
        return colour_query

    def hasSizeQuery(self):
        """ b0, b1, b2"""
        """small, medium, large"""
        sizes = ["small","medium","large"]
        size_query = []
        for b_id, block in enumerate(self.blocks):
            for s in sizes:
                if s == block.getSize():
                    query = "true"#["has_size("+str(b_id)","+block.getSize()+")", "true"]
                else:
                    query = "false"#["has_size("+str(b_id)","+block.getSize()+")", "false"]
                size_query.append(query)
        return size_query
