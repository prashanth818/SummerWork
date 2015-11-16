import pygame
from std_msgs.msg import String
from copy import deepcopy
class Screen:
    """Handles display."""
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.size = (self.width, self.height)
        self.running = True
        self.disp_surf = None
        self.background = (255,255,255)

    def on_init(self):
        pygame.init()
        print pygame.display.get_init()
        self.disp_surf = pygame.display.set_mode(self.size)
        self.running = True

    def on_event(self, event):
        if event.type == pygame.QUIT:
            self.running = False


    def on_render(self, blocks, arm, boundaries, string, bool_): #, rule_box):
        self.disp_surf.fill(self.background)
        for boundary in boundaries:
            pygame.draw.line(self.disp_surf, (0,0,0),boundary[0], boundary[1], 4)
        for block in blocks:
            block.renderShape(self.disp_surf)

        arm.renderShape(self.disp_surf)
        # rule_box.renderShape(self.disp_surf)

        font = pygame.font.SysFont("monospace", 30)
        label = [0]
        if len(string) != 1:
            label = [0]*(len(string)-3)
        lineCount = 0
        gg = []
        for i, st in enumerate(string):
            
            if string[i] in ['-1', '0', '1', '2']:
                gg.append([int(string[i]), (i-lineCount), 0, 0])  # destBlock, blockid, location
            else:
                lineCount +=1
                label[i] = font.render(str(string[i]), 1, (0,0,0))
        gg.sort(key=lambda x: x[0])
        # print gg
        for i, st in enumerate(label):
            self.disp_surf.blit(st, (720, 50*i))

        tableCount = 0
        hh = gg
        for element in gg:
            # print element
            if element[0] == -1: # if on table
                label = font.render(str(element[1]), 1, (0,0,0))
                self.disp_surf.blit(label, (720+(50*tableCount), 360))
                element[2] = 720+(50*tableCount)
                element[3] = 360
                tableCount +=1
            else:
                label = font.render(str(element[1]), 1, (0,0,0))
                for el in hh:
                    if el[1] == element[0]:  #block id == location
                        destBlock = el
                        self.disp_surf.blit(label, (destBlock[2], destBlock[3]-30)) #same x, higher y
                        element[2] = destBlock[2]
                        element[3] = destBlock[3]-30
        if bool_:
            label = font.render( "-occurs(has_shape(D, prism), has_colour(D, red))", 1, (0,0,0))
            self.disp_surf.blit(label, (100, 500))
           




        pygame.display.update()
    def on_cleanup(self):
        pygame.quit()
