class RuleBox:
    def __init__(self, position):
        self.position = position
        self.width = 1022
        self.height = 300
        self.rules = []

    def updateRules(self, rules):
        self.rules = rules
        
    def renderShape(self, surface):
        base_coord = self.createBase()
        pygame.draw.polygon(surface, self.colour, base_coord)
        pincher_coord = self.createPinchers()
        pygame.draw.rect(surface, self.colour, pincher_coord[0])
        pygame.draw.rect(surface, self.colour, pincher_coord[1])
