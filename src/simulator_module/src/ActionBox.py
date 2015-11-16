import pygame
class ActionBox:
    def __init__(self, position, width):
        self.position = position
        self.width = width
        self.height = height
        self.action = None

    def updateRules(self, rules):
        self.rules = rules
    def renderFont(self, surface):
        pass
