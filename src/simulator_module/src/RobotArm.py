import pygame
class RobotArm:
    def __init__(self, base_position, pincher_seperation, boundaries):
        self.base_width = 100
        self.base_height = 30
        self.pincher_width = 10
        self.pincher_height = 50
        self.colour = (0,0,255)
        self.base_position = base_position
        self.pincher_seperation = pincher_seperation
        self.MAX_PINCHER_SEPERATION = self.base_width - self.pincher_width
        self.direction = []
        self.holding = None
        self.base_rect = pygame.Rect(self.base_position[0], self.base_position[1], self.base_width, self.base_height)
        self.pincher_positions = self.createPinchers()
        (lpx, lpy, lpw, lph), (rpx, rpy, rpw, rph) = self.pincher_positions
        self.lp_rect = pygame.Rect(lpx, lpy, lpw, lph)
        self.rp_rect = pygame.Rect(rpx, rpy, rpw, rph)
        self.grabbing = False

    def getDirection(self, position):
        if self.base_position[0] == position[0] and self.base_position[1] == position[1]:
            self.direction = []
        else:
            self.direction.append((position[1] - self.base_position[1])/(position[0] - self.base_position[0]))
            self.direction.append(position[1] - self.direction[0]*position[0])

    def move(self, direction):

        if direction == "Right":
            self.base_position[0] += 1
        elif direction == "Left":
            self.base_position[0] -= 1
        if direction == "Up":
            self.base_position[1] -= 1
        elif direction == "Down":
            self.base_position[1] += 1
        self.base_rect = pygame.Rect(self.base_position[0], self.base_position[1], self.base_width, self.base_height)
        self.pincher_positions = self.createPinchers()
        (lpx, lpy, lpw, lph), (rpx, rpy, rpw, rph) = self.pincher_positions
        self.lp_rect = pygame.Rect(lpx, lpy, lpw, lph)
        self.rp_rect = pygame.Rect(rpx, rpy, rpw, rph)

    def grab(self, direction):
        if self.pincher_seperation <=  self.base_width-self.pincher_width:
            if self.pincher_seperation >= self.pincher_width:
                if direction == "Close" and self.grabbing == False:
                    self.pincher_seperation -= 1
                elif direction == "Open":
                    self.pincher_seperation += 1
            else:
                self.pincher_seperation = self.pincher_width
        else:
            self.pincher_seperation = self.base_width-self.pincher_width
        self.pincher_positions = self.createPinchers()
        (lpx, lpy, lpw, lph), (rpx, rpy, rpw, rph) = self.pincher_positions
        self.lp_rect = pygame.Rect(lpx, lpy, lpw, lph)
        self.rp_rect = pygame.Rect(rpx, rpy, rpw, rph)
    def createBase(self):
        width = self.base_width
        height = self.base_height
        return (self.base_position[0], self.base_position[1], width, height)

    def createPinchers(self):
        #left pincher
        node_1 = (self.base_position[0], self.base_position[1]+self.base_height,self.pincher_width, self.pincher_height)
        #right pincher
        node_2 = (self.base_position[0] + self.pincher_seperation, self.base_position[1]+self.base_height, self.pincher_width, self.pincher_height)
        return (node_1, node_2)

    def renderShape(self, surface):
        rect_attr = self.createBase()
        pygame.draw.rect(surface, self.colour, rect_attr)
        pincher_coord = self.createPinchers()
        pygame.draw.rect(surface, self.colour, pincher_coord[0])
        pygame.draw.rect(surface, self.colour, pincher_coord[1])
