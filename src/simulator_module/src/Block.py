import pygame
class Block:
    def __init__(self, index, shape, colour, size, position, boundaries):
        self.index = index
        self.shape = shape
        self.colour = colour
        self.size = size
        self.position = position
        self.home_position = self.position
        self.boundaries = boundaries
        self.rect = pygame.Rect(position[0], position[1], self.shape.width, self.shape.height)
        self.direction = []
        self.supported = False
        self.grabbed = False
        self.grabdist = 10
    def getIsGrabbed(self):
        return self.grabbed

    def setGrabbed(self, grabbed):
        self.grabbed = grabbed

    def getShape(self):
        return self.shape

    def getPosition(self):
        return self.position

    def move(self, direction):
        if direction == "Right":
            self.position[0] += 1
        elif direction == "Left":
            self.position[0] -= 1
        elif direction == "Up":
            self.position[1] -= 1
        elif direction == "Down":
            self.position[1] += 1
        self.rect = pygame.Rect(self.position[0], self.position[1], self.shape.width, self.shape.height)

    def renderShape(self, surface):
        if self.shape.shape == "prism":
            node_1 = self.position[0], self.position[1] + self.shape.height
            node_2 = self.position[0] + self.shape.width, self.position[1] + self.shape.height
            node_3 = self.position[0] + self.shape.width/2, self.position[1]
            pygame.draw.polygon(surface, self.colour,(node_1, node_2, node_3))
        else:
            pygame.draw.rect(surface, self.colour, (self.position[0], self.position[1], self.shape.width, self.shape.height))
