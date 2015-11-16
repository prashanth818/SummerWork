# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 16:24:02 2015

@author: Prashanth
"""

class Block:
    def __init__(self, label, shape, colour, size):
        self.label = label
        self.shape = shape
        self.colour = colour
        self.size = size
        """END"""
    def getLabel(self):
        return self.label
    def getShape(self):
        return self.shape
    def getSize(self):
        return self.size
    def getColour(self):
        return self.colour
