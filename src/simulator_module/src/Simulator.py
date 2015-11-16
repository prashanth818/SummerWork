#! /usr/bin/env python
import rospy
import pygame
import time
import random as r
from Shape import Shape
from Block import Block as B
from RobotArm import RobotArm
from RuleBox import RuleBox
from Screen import Screen
from std_msgs.msg import String
from asp_module.msg import *
from simulator_module.srv import *
from simulator_module.msg import *
from controller_module.srv import *
import re


class BlocksWorld:
    def __init__(self):
        self.screen = Screen(1024, 768)
        self.blocks = []
        self.shape = ("cube", "prism", "cuboid")
        self.colour = ("red", "blue", "green")
        self.size = ("small" , "medium", "large")
        self.boundaries = [((0,402), (1024,402)),((700,0),(700,400))]
        self.table = 400
        self.wall = 700
        self.arm = RobotArm([100,100], 50, self.boundaries)
        self.colliding_blocks = []
        self.state = -1
        self.ablock = None
        self.dblock = None
        self.big_block = False
        self.config_flag = False
        self.start_config = False
        self.send = False
        self.stringsToPrint = []
        rospy.init_node('simulator', anonymous=True)
        srv_addAction = rospy.Service('SimulatorAddAction', SimulatorAddAction, self.getAction)
        self.print_suscriber = rospy.Subscriber('/controller/print', Data, self.thingsToPrint)
        self.print_suscriber2 = rospy.Subscriber('/controller/print2', String, self.thingsToPrint2)
        self.print_suscriber3 = rospy.Subscriber('/controller/print3', String, self.thingsToPrint3)
        self.failed = False
        self.iterations = 0

    def initialise_Simulator(self):
        self.colliding_blocks = []
        self.state = -1
        self.blocks = []
        self.makeRandomBlocks()
        config = self.getConfiguration()
        config_msg = Configuration(config = config)
        block_msg = []
        colour_vars = {(255,0,0):"red", (0,255,0):"green", (0,0,255):"blue"}
        for block in self.blocks:
            block_colour = colour_vars[block.colour]
            block_msg.append(Block(label = "block"+str(block.index), shape = block.shape.shape, colour = block_colour, size = block.size))
        return [config_msg, block_msg]


    def sendState(self, current_state):
#         """ This will send state to controller"""
        rospy.wait_for_service('AddCurrentState')
        try:
            sendState = rospy.ServiceProxy('AddCurrentState', AddCurrentState)
            state = sendState(current_state)
            # configuration = Configuration(state.state.configuration.config)
            # self.currentState = State(configuration = configuration, block_properties = state.state.block_properties)
        except rospy.ServiceException, e:
            print "Service call failed %s" % e

    def makeRandomBlocks(self):
        shape_vars = {"prism": [30,30], "cube":[30,30], "cuboid" : [60,30]}
        size_vars = {"small": [-5,-5], "medium": [5,5],"large": [30, 30]}
        colour_vars = {"red": (255,0,0), "green":(0,255,0), "blue":(0,0,255)}
        for i in range(0,3):
            rand_shape = self.shape[i]#r.choice(self.shape)

            rand_colour = self.colour[i]
            rand_size = self.size[1]#r.choice(self.size)

            width = shape_vars[rand_shape][0] + size_vars[rand_size][0]
            height = shape_vars[rand_shape][1] + size_vars[rand_size][1]
            colour = colour_vars[rand_colour]
            block_shape = Shape(rand_shape, (width, height))
            position = [150*i + r.randint(75,100), 400 - height]

            self.blocks.append(B(i,block_shape, colour, rand_size, position, self.boundaries[0]))

    def blockOverlapCheck(self):
        """Do not show this to anyone"""
        self.colliding_blocks = []
        for block in self.blocks:
            for another_block in self.blocks:
                if block.index != another_block.index:
                    if block.rect.colliderect(another_block.rect):
                        if not self.colliding_blocks:
                            if block.index < another_block.index:
                                self.colliding_blocks.append((block, another_block))
                            else:
                                self.colliding_blocks.append((another_block, block))
                        else:
                            flag = False
                            for t in self.colliding_blocks:
                                if block == t[0] and another_block == t[1]:
                                    flag = True
                                    break
                                elif block == t[1] and another_block == t[0]:
                                    flag = True
                                    break
                                else:
                                    flag = False
                            if flag == False:
                                if block.index < another_block.index:
                                    self.colliding_blocks.append((block, another_block))
                                else:
                                    self.colliding_blocks.append((another_block, block))

    def armCollide(self):
        (lpx, lpy, lpw, lph), (rpx, rpy, rpw, rph) = self.arm.pincher_positions
        abx, aby = self.arm.base_position
        abw, abh = (self.arm.base_width, self.arm.base_height)
        for block in self.blocks:
            bx, by = block.position
            bw, bh = block.shape.dimensions
            if block.shape.width < self.arm.pincher_seperation-rpw-7:
                self.arm.grabbing = False
                block.setGrabbed(False)
            if block.shape.shape == "prism":
                if self.arm.base_rect.colliderect(block.rect):
                    if by < aby+abh:
                        self.arm.move("Up")
                if self.arm.lp_rect.colliderect(block.rect):
                    (l_m, l_c), (r_m, r_c) = self.prismBlockCollision(block)

                    if lpx <= bx+bw/2 and lpx + lpw >=  bx+bw/2:
                        if lpy + lph > by:
                            self.arm.move("Up")
                    else:
                        if lpy+lph > by+bh*4/5:
                            if lpx > bx + bw/2:
                                right_side = (lpy+lph - r_c)/r_m
                                if lpx < right_side and lpx+lpw >= right_side:
                                    block.move("Left")
                                    # self.arm.move("Right")
                            elif lpx + lpw < bx + bw/2:
                                left_side = (lpy+lph - l_c)/l_m
                                if lpx+lpw > left_side and lpx <= left_side:
                                    if block.shape.width >= self.arm.pincher_seperation-lpw+2:
                                        block.setGrabbed(True)
                                        self.arm.grabbing = True

                if self.arm.rp_rect.colliderect(block.rect):
                    (l_m, l_c), (r_m, r_c) = self.prismBlockCollision(block)
                    if rpx <= bx+bw/2 and rpx + rpw >=  bx+bw/2:
                        if rpy + rph > by:
                            self.arm.move("Up")
                    else:
                        if rpy+rph > by+bh*4/5:
                            if rpx > bx + bw/2:
                                right_side = (rpy+rph - r_c)/r_m
                                if rpx < right_side and rpx+rpw >= right_side:
                                    if block.shape.width >= self.arm.pincher_seperation-rpw+2:
                                        block.setGrabbed(True)
                                        self.arm.grabbing = True
                                    # else:
                                    #     block.move("Left")
                                        # self.arm.move("Right")

                            elif rpx + rpw < bx + bw/2:
                                left_side = (lpy+lph - l_c)/l_m
                                if rpx+rpw > left_side and rpx <= left_side:
                                    block.move("Right")
                                    # self.arm.move("Left")

            else:
                if self.arm.base_rect.colliderect(block.rect):
                    if by < aby+abh:
                        self.arm.move("Up")
                elif self.arm.lp_rect.colliderect(block.rect):
                    if by > lpy+lph-bh/8:
                        self.arm.move("Up")
                    elif bx+bw >= lpx and bx < lpx:
                        block.move("Left")
                        # self.arm.move("Right")
                    elif bx <= lpx+lpw and bx+bw > lpx:
                        if block.shape.width >= self.arm.pincher_seperation-rpw:
                            block.setGrabbed(True)
                            self.arm.grabbing = True
                        else:
                            # self.arm.move("Left")
                            block.move("Right")
                elif self.arm.rp_rect.colliderect(block.rect):
                    if by > rpy+rph-bh/8:
                        self.arm.move("Up")
                    elif rpx + rpw >= bx and rpx < bx:
                        # self.arm.move("Left")
                        block.move("Right")
                    elif bx + bw >= rpx and bx < rpx:
                        if block.shape.width >= self.arm.pincher_seperation-rpw:
                            block.setGrabbed(True)
                            self.arm.grabbing = True
                        else:
                            # self.arm.move("Right")
                            block.move("Left")

    def prismBlockCollision(self, block):
        (lpx, lpy, lpw, lph), (rpx, rpy, rpw, rph) = self.arm.pincher_positions
        abx, aby = self.arm.base_position
        abw, abh = (self.arm.base_width, self.arm.base_height)
        bx, by = block.position
        bw, bh = block.shape.dimensions
        #left line
        left_m = -2*bh/bw
        left_c = (by) - left_m*(bx+bw/2)
        #right line
        right_m = 2*bh/bw
        right_c = (by) - right_m*(bx+bw/2)
        return ((left_m, left_c), (right_m, right_c))

    def avoidBoundaries(self):
        for block in self.blocks:
            bx, by = block.position
            if by + block.shape.height > self.table:
                block.move("Up")
                block.move("Up")
                block.move("Up")
            elif bx <= 1:
                block.move("Right")
            elif bx + block.shape.width >= self.wall:
                block.move("Left")

    def naturalFall(self):
        for block in self.blocks:
            bx, by = block.position
            if block.getIsGrabbed() == False:
                if by + block.shape.height < self.table:
                    if block.supported == False:
                        block.move("Down")
                        block.move("Down")
                        block.move("Down")
            else:
                (lpx, lpy, lpw, lph), (rpx, rpy, rpw, rph) = self.arm.pincher_positions
                x = lpx+lpw
                y = lpy+lph-block.shape.height
                block.position = [x,y]
                block.rect = pygame.Rect(block.position[0], block.position[1], block.shape.width, block.shape.height)


    def moving_logic(self, block1, block2):
        b1x, b1y = block1.position
        b2x, b2y = block2.position
        if b1y + block1.shape.height > b2y and b1y < b2y:
            if block2.shape.shape != "prism":
                block1.supported = True
            else:
                block1.supported = False
        else:
            block1.supported = False
        falling = self.topBlockIsFalling(block1, block2)
        b1_on_right = self.b1OnRight(block1, block2)
        # b2_on_right = self.b2OnRight(block1, block2)
        if falling == True:
            if b1_on_right == True:
                block1.move("Right")
                block2.move("Left")
            else:
                block1.move("Left")
                block2.move("Right")
        else:
            if b1y == b2y:
                if (b1x + block1.shape.width) > b2x and b1x <= b2x:
                    block1.move("Left")
                    block2.move("Right")
                elif (b2x + block2.shape.width) > b1x and b2x <= b1x:
                    block2.move("Left")
                    block1.move("Right")
                else:
                    block2.move("Left")
                    block1.move("Right")

    def topAndBottomBlock(self, block1, block2):
        b1x, b1y = block1.position
        b2x, b2y = block2.position
        if b1y < b2y:
            topBlock = block1
            bottomBlock = block2
        elif b1y > b2y:
            topBlock = block2
            bottomBlock = block1
        else:
            topBlock = None
            bottomBlock = None
        return (topBlock, bottomBlock)

    def topBlockIsFalling(self, block1, block2):
        topBlock, bottomBlock = self.topAndBottomBlock(block1, block2)
        if topBlock != None and bottomBlock != None:
            b1x, b1y = topBlock.position
            b2x, b2y = bottomBlock.position
            if bottomBlock.shape.shape == "prism":
                return True
                topBlock.supported = False
            else:
                if topBlock.shape.width < bottomBlock.shape.width:

                    b1_on_right = self.b1OnRight(topBlock, bottomBlock)
                    if b1_on_right:
                        thrs = float(b2x - b1x + block2.shape.width)/float(block1.shape.width)
                    else:
                        thrs = float(b1x + block1.shape.width -b2x)/float(block1.shape.width)
                    if thrs > 0.5:
                        return False
                    else:
                        return True
                        topBlock.supported = False
                else:
                    t_mid = b1x + topBlock.shape.width/2
                    b_mid = b2x + bottomBlock.shape.width/2
                    thrs = abs(t_mid - b_mid)/float(bottomBlock.shape.width)
                    if thrs < 0.10:
                        return False
                    else:
                        return True
                        topBlock.supported = False
        else:
            return False

    def b2OnRight(self, block1, block2):
        b1x, b1y = block1.position
        b2x, b2y = block2.position
        b1w, b1h = block1.shape.dimensions
        b2w, b2h = block2.shape.dimensions
        if (b1x + b1w/2.0) < (b2x + b2w/2.0):
            return True
        else:
            return False

    def b1OnRight(self,block1, block2):
        b1x, b1y = block1.position
        b2x, b2y = block2.position
        b1w, b1h = block1.shape.dimensions
        b2w, b2h = block2.shape.dimensions
        if (b1x + b1w/2.0) >= (b2x + b2w/2.0) :
            return True
        else:
            return False

    def avoidCollisions(self):
        if len(self.colliding_blocks) > 0:
            for block_pairs in self.colliding_blocks:
                block1, block2 = block_pairs
                topBlock, bottomBlock = self.topAndBottomBlock(block1, block2)
                if topBlock != None and bottomBlock != None:
                    self.moving_logic(topBlock, bottomBlock)
                else:
                    self.moving_logic(block1, block2)

    def getConfiguration(self):
        config = []
        for block in self.blocks:
            bx, by = block.position
            if by + block.shape.height >= self.table:
                config.append(-1)
            elif block.grabbed == True:
                config.append(-2)
            else:
                temp = None
                for dest_block in self.blocks:
                    dx, dy = dest_block.position
                    if block.index != dest_block.index:
                        if bx + block.shape.width/2 > dx and bx + block.shape.width/2 < dx + dest_block.shape.width:
                            if block.supported == True and by < dy:
                                if temp == None or temp.position[1] > dy:
                                    temp = dest_block
                config.append(temp.index)

        return config

    def armControl(self):
        armx, army = self.arm.base_position
        (lpx, lpy, lpw, lph), (rpx, rpy, rpw, rph) = self.arm.pincher_positions
        p_s = self.arm.pincher_seperation
        """
        Temporary control of arm. For testing and debugging purposes only.
        Real arm must be automated and not require user input.
        """
        if armx > 1 and armx + self.arm.base_width < self.wall:
            if army > 1 and army + self.arm.base_height + self.arm.pincher_height <= self.table:
                if self.state == 0:
                    self.config_flag = False
                    if army > 100:
                        self.arm.move("Up")
                    else:
                        self.state = 1
                        print self.state
                if self.state == 1:
                    self.arm.grab("Open")
                    if lpx+lpw > self.ablock.position[0]:
                        self.arm.move("Left")
                    elif lpx+lpw < self.ablock.position[0]:
                        self.arm.move("Right")
                    else:
                        self.state = 2
                        print self.state
                if self.state == 2:
                    if self.ablock.shape.shape == "prism":
                        if lpy+lph < self.ablock.position[1]+self.ablock.shape.height*0.85:
                            self.arm.move("Down")
                            if self.ablock.size == "large":
                                if army+ self.arm.base_height >= self.ablock.position[1]:
                                    self.state = 7
                                    print self.state
                        else:
                            self.state = 3
                            print self.state
                    else:
                        if self.ablock.size == "large":
                            if self.ablock.shape.shape == "cuboid":
                                if lpy+lph >= self.ablock.position[1]:
                                    self.state = 7
                                    print self.state
                                else:
                                    self.arm.move("Down")
                            elif self.ablock.shape.shape == "cube":
                                if army+ self.arm.base_height >= self.ablock.position[1]:
                                    self.state = 7
                                    print self.state
                                else:
                                    self.arm.move("Down")
                        else:
                            if lpy+lph < self.ablock.position[1]+self.ablock.shape.height/2:
                                self.arm.move("Down")
                            else:
                                self.state = 3
                                print self.state
                if self.state == 3:
                    if self.ablock.grabbed == False:
                        if p_s + rpw >= self.ablock.shape.width:
                            self.arm.grab("Close")
                        elif p_s + rpw  < self.ablock.shape.width:
                            self.arm.grab("Open")
                    else:
                        self.state = 4
                        print self.state
                if self.state == 4:
                    if army > 100:
                        self.arm.move("Up")
                    else:
                        print self.dblock
                        print self.send
                        if self.dblock != None:
                            self.state = 5


# <<<<<<< HEAD
                        if self.send == False:
                            state = self.sendCurrentState()
# =======
#                             print self.send
#                         if self.send == False:
#                             state = self.sendCurrentState()
#                             print (state, "SOdfalskdfjalskdfjlkj")
# >>>>>>> 394421bd659ef044222d492354a1fbebb5a7f01e



                            self.send = True
                            self.sendState(state)

                        # Call Controller service to send state
                    ### GIve configuration here
                    ### Waits for action

                if self.state == 5:
                    self.send = False
                    self.config_flag = False
                    if self.dblock != "tab0":
                        if self.ablock.position[0] + self.ablock.shape.width/2 > self.dblock.position[0] + self.dblock.shape.width/2:
                            self.arm.move("Left")
                        elif self.ablock.position[0] + self.ablock.shape.width/2 < self.dblock.position[0] + self.dblock.shape.width/2:
                            self.arm.move("Right")
                        else:
                            self.state = 6
                            print self.state
                    else:
                        if self.ablock.position[0] > self.ablock.home_position[0]:
                            self.arm.move("Left")
                        elif self.ablock.position[0] < self.ablock.home_position[0]:
                            self.arm.move("Right")
                        else:
                            self.state = 6
                if self.state == 6:
                    if self.dblock != "tab0":
                        if self.ablock.position[1] + self.ablock.shape.height >= self.dblock.position[1]:
                            if p_s <= self.arm.MAX_PINCHER_SEPERATION:
                                self.arm.grab("Open")



# <<<<<<< HEAD
                                self.ablock.supported = False
# =======
# >>>>>>> 394421bd659ef044222d492354a1fbebb5a7f01e


                            else:
                                self.state = 7
                                print self.state
                        else:
                            self.arm.move("Down")
                    else:
                        if self.ablock.position[1] >= self.ablock.home_position[1]:
                            if p_s <= self.arm.MAX_PINCHER_SEPERATION:
                                self.arm.grab("Open")
                            else:
                                self.state = 7
                                print self.state
                        else:
                            self.arm.move("Down")
                if self.state == 7:
                    if army > 100:
                        self.arm.move("Up")
                    else:
                        self.state = -1
                        self.iterations += 1
                        print self.state
                        state = self.sendCurrentState()
                        self.sendState(state)
                        #Call controller service
                if self.state == -1:
                    self.config_flag = False
                    self.ablock = None
                    self.dblock = None
                    if self.start_config == True:
                        config_msg, block_msg = self.initialise_Simulator()
                        print config_msg
                        self.sendState(State(configuration = config_msg, block_properties = block_msg))
                        self.start_config = False


            elif army <= 1:
                self.arm.move("Down")
            elif army + self.arm.base_height + self.arm.pincher_height > self.table:
                self.arm.move("Up")
        elif armx <= 1:
            self.arm.move("Right")
        elif armx + self.arm.base_width >= self.wall:
            self.arm.move("Left")

    def on_loop(self):
        self.blockOverlapCheck()
        self.avoidCollisions()
        self.avoidBoundaries()
        self.naturalFall()
        self.armControl()
        self.armCollide()

    def getAction(self, action):
        # block = raw_input("Enter Blocks: ")
        print action
        ab = None
        db = None
        if action.action.actionableBlock == "null" and action.action.destinationBlock == "null":
            print "gets set true"
            self.start_config = True
        else:
            if self.state == -1 and action.action.actionableBlock != "null":
                ab = int(action.action.actionableBlock[-1])
                self.state = 0
            elif self.state == 4:
                if action.action.destinationBlock == "tab0":
                    db = -1
                elif action.action.destinationBlock != "null":
                    db = int(action.action.destinationBlock[-1])
            #db = blocks[1]
            for b in self.blocks:
                if db != -1:
                    if ab != db:
                        if ab == b.index:
                            self.ablock = b
                        if db == b.index:
                            self.dblock = b
                else:
                    self.dblock = "tab0"
        return 1

    def sendCurrentState(self):
        config_msg = Configuration(config = self.getConfiguration())
        block_msg = []
        colour_vars = {(255,0,0):"red", (0,255,0):"green", (0,0,255):"blue"}
        for block in self.blocks:
            block_colour = colour_vars[block.colour]
            block_msg.append(Block(label = "block"+str(block.index), shape = block.shape.shape, colour = block_colour, size = block.size))
        return State(configuration = config_msg, block_properties = block_msg)

    def thingsToPrint(self, data):
        print "*************"
        print data.goal
        print data.action
        print "*************"

        self.stringsToPrint = []

        self.stringsToPrint.append("Action:")
        self.stringsToPrint.append(data.action.action)
        self.stringsToPrint.append(data.action.actionableBlock)
        if data.action.destinationBlock == "tab0":
            self.stringsToPrint.append("table")
        elif data.action.destinationBlock != "null0":
            self.stringsToPrint.append(data.action.destinationBlock)
        if data.action.goalAchieved:
            self.stringsToPrint.append("Goal: ACHEIVED")
        else:
            self.stringsToPrint.append("Goal:")
        for block in list(data.goal.config):
            self.stringsToPrint.append(str(block))

    def thingsToPrint2(self, data):
        self.stringsToPrint = []
        self.stringsToPrint.append(data)

    def thingsToPrint3(self, state):
        print "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"
        self.failed = True


    def on_execute(self):
        self.screen.on_init()
        # self.makeRandomBlocks()
        bool_ = False
        self.screen.on_render(self.blocks, self.arm, self.boundaries, self.stringsToPrint, bool_)
        while(self.screen.running):
            for event in pygame.event.get():
                self.screen.on_event(event)
            self.on_loop()
            print self.iterations
            print self.failed
            if self.iterations >5 and self.failed == True:
                bool_ = True
            self.screen.on_render(self.blocks, self.arm, self.boundaries, self.stringsToPrint, bool_)
            pygame.time.wait(3)

        self.screen.on_cleanup()

if __name__ == "__main__":


    print "Ready to Simulate!"
    display = BlocksWorld()
    display.on_execute()
    # rospy.spin()


        # shape_vars = {"prism": [30,30], "cube":[30,30], "cuboid" : [60,30]}
        # size_vars = {"small": [-5,-5], "medium": [5,5],"large": [30, 30]}
        # colour_vars = {"red": (255,0,0), "green":(0,255,0), "blue":(0,0,255)}
