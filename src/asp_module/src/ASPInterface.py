#! /usr/bin/env python
import subprocess
import rospy
import os
import re
import tf
from asp_module.srv import *
from asp_module.msg import *
from threading import Thread
from std_msgs.msg import Int16


# TODO: move files to common



class ASPInterface:
    """Used to interact withe the ASP Knowledge Base"""

    def __init__(self, solver_path, asp_path):

        # Stores rules generated in this timestep. Useful so we can look for rule duplicates/conflicts
        self.solver_path = solver_path
        self.asp_path = asp_path
        self.current_plan = []
        self.goal = 'null'
        self.iterator = 0
        self.timestep = 0
        self.observations = []
        self.tableSurface = 0

        #TODO: add lauch file with parameter solver path and dlv path

        rospy.init_node('AspServer')
        # srv_query = rospy.Service('AspQuery', AspQuery, self.queryHandler)
        srv_addObs = rospy.Service('AspAddObservation', AspAddObservation, self.addObservationHandler)
        srv_answer = rospy.Service('AspAnswer', AspAnswer, self.answerHandler)
        srv_state = rospy.Service('AspCurrentState', AspCurrentState, self.addNewBlocks)


    def solve(self, mode, timeStep, pfilter='', goal=''):
        """
        Solves the current knowledge base and produces answer set
        :param solverPath: location/name of the asp solver being used
        :param aspPath: location/name of merged asp file
        :return: location/name of the outputted answer set
        """
        # merge files to create kb
        self.merge(mode, timeStep)

        if pfilter:
            pfilter = '-solveropts "-pfilter=' + pfilter + '"'

        fpath_answer = os.path.join(os.path.dirname(__file__), 'asp.answer')  # Where the answer set is written out
        proc = subprocess.Popen(['/bin/bash'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        stdout = proc.communicate('java -jar ' + self.solver_path + '/sparc.jar ' + self.asp_path + '/merged.sp -solver dlv -A ' + pfilter + ' > ' + fpath_answer)

        return fpath_answer


    def merge(self, mode, timeStep):
        """
        Grabs all the different bits of asp and merges to a single file
        :return:
        """
        try:

            fpath_constants = os.path.join(os.path.dirname(__file__), 'constants.sp')
            fpath_rules = os.path.join(os.path.dirname(__file__),'rules.sp')
            fpath_initial = os.path.join(os.path.dirname(__file__),'initial.sp')
            fpath_config = os.path.join(os.path.dirname(__file__),'test.sp')
            fpath_history = os.path.join(os.path.dirname(__file__),'history.sp')
            fpath_output = os.path.join(os.path.dirname(__file__),'merged.sp')
            fpath_planner= os.path.join(os.path.dirname(__file__), 'planning.sp')
            fpath_explainer= os.path.join(os.path.dirname(__file__), 'explanation.sp')
            fpath_goal = os.path.join(os.path.dirname(__file__),'goal.sp')

            filenames = [fpath_constants, fpath_rules, fpath_initial, fpath_config, fpath_history]

            if mode == 'planning':
                filenames.append(fpath_planner)
                # if goal != '':
                #     filenames.append(fpath_goal)
            elif mode == 'explaining':
                filenames.append(fpath_explainer)


            with open( fpath_output, 'w') as outfile:
                for fname in filenames:
                    with open(fname) as infile:
                        outfile.write(infile.read())
                    if fname is fpath_constants:
                        outfile.write('#const numSurfaces = ' + str(self.tableSurface) + '.\n')  #tableSurface is the number of surfaces we need
                        outfile.write('#const numBlocks = ' + str(self.tableSurface-1) + '. \n')
                        outfile.write("#const numSteps = " + str(timeStep) + ".\n")
                    elif fname is fpath_planner:
                        outfile.write(self.goal)
        except:
            print "Merging ASP files failed"

    def addObservationHandler(self, observation):
        pass

    def addNewBlocks(self, state):
        """ This function discards the previous block
        configuration and generates the ASP rules for
        the given state"""

        surface  = 0
        config = state.current_state.configuration.config
        size = len(state.current_state.block_properties)
        self.tableSurface = size
        print "initial"
        print config

        fpath_config = os.path.join(os.path.dirname(__file__),'test.sp')
        with open( fpath_config, 'w') as outfile:
            outfile.write('has_surface(tab0, s' + str(size) + ').\n')

            for block in state.current_state.block_properties:
                print surface
                outfile.write('has_size('+ block.label +', ' + block.size + ').\n')
                outfile.write('has_colour('+ block.label +', ' + block.colour + ').\n')
                outfile.write('has_shape('+ block.label +', ' + block.shape + ').\n')
                outfile.write('has_surface('+ block.label +', s' + str(surface)+ ').\n')

                if config[surface] == -1: #error IndexError: tuple index out of range
                    outfile.write('holds(on(' + block.label + ', s' + str(size) + '), 0).\n')
                elif config[surface] == -2:
                    outfile.write('holds(has_object(rob0, '+ block.label +'),0).\n')
                else:
                    outfile.write('holds(on(' + block.label + ', s' + str(config[surface]) + '), 0).\n')
                surface = surface +1

        # if all is ok
        return True

    def querySurface(self, surface):
        """ This function takes a surface and returns
        the object that this surface belongs to"""
        try:
            pattern = 'has_surface(.*?, ' + surface + ').'
            fpath_initial = os.path.join(os.path.dirname(__file__), 'test.sp')
            for line in open(fpath_initial):
                for match in re.findall(pattern, line):
                    # isolate block
                    block = re.findall("\((.*),", match)[0]
                    return block
        except:
            print "Error in querySurface()"


    def answerHandler(self, goal=''):
        """ Function called when a node calls AspAnswer service.
        Attempts to generate a plan for the goal that is given."""

        string =  'goal(I) :- '
        index = 0
        print 'goal'
        print goal.goal.config
        for block in goal.goal.config:
            string +=  ' holds(on(block' + str(index) + ', s'
            if block == -1:
                string += str(self.tableSurface) + '), I),'
            else:
                string += str(block) + '), I),'
            index += 1
        string = string[:-1] + '.'


        if string != self.goal:
            try:
                print 'new goal!'
                self.goal = string
                self.iterator = 0
                timestep = self.timestep
                max_timestep = 17
                #TODO make this dependant on the num of blocks

                while(timestep <max_timestep):
                     #  Solve and read
                    fpath_answer = self.solve('planning', timestep, pfilter='occurs')
                    with open(fpath_answer, 'r') as infile:
                        raw = infile.read()
                        self.parse_answer(raw, timestep)
                    if not self.current_plan:
                        timestep += 1
                    else:
                        return AspAnswerResponse(parsed=self.current_plan[self.iterator])
            except:
                print "An error occured in answerHandler"
        else:
            try:
                print 'same goal!'
                self.iterator += 1
                action = self.current_plan[self.iterator]
                print action
                return AspAnswerResponse(parsed= action)
            except IndexError:
                return AspAnswerResponse(parsed= Action(action = 'null', actionableBlock = 'null', destinationBlock = 'null', timestep=0, config = Configuration([]), goalAchieved = True))



    def parse_answer(self, raw, timestep):
        """ This parses an answer set, looking for actions with blocks"""
        # try:
        max_timestep = timestep
        parsed =[] # Use this to save list of SubGoals
        if len(raw) <2:
            pass
        else:
            raw = re.findall("\{(.*?)\}", raw)[0]  # The answer set is inside the squiggly brackets {}
            # remove spaces between steps
            raw = raw.replace(' ', '')
            # remove occurs and save steps in a list
            anslist = raw.split('occurs')

            self.solve('planning', timestep, pfilter='holds')

            for step in anslist:
                if step:
                    # get action
                    action = re.findall("\((.*?)\(", step)[0]
                    # get the timestep
                    timestep = re.findall("\)(.*)\)", step)
                    # remove the comma from timestep
                    timestep = timestep[0].replace(',', '')
                    # remove the outer brackets from statement
                    temp = re.search("\((.*)\)", step).group(1)
                    # remove brackets
                    target = re.findall("\((.*)\),", temp)[0]
                    # put arguments into a list
                    arglist = target.split(',')
                    # we can ignore first element for now, as we are only dealing with one agent
                    block = arglist[1]
                    if action == 'put_down':
                        surface = arglist[2]
                        # find out which object the surface belongs to
                        destBlock = self.querySurface(surface)
                    else:
                        destBlock = 'null'


                    # self.AspToConfig(self.getExpectedConfig(timestep))

                    parsed.insert(int(timestep),(Action(action = action, actionableBlock = block, destinationBlock = destBlock, timestep=int(timestep), config =self.getExpectedConfig(timestep), goalAchieved = False)))

                    
        parsed.sort(key=lambda x: int(x.timestep))
        self.current_plan = parsed
        return
        # except:
            # print 'error occured in parsing'


    def newTimestepCallback(self, timestep):
        """New timestep"""
        print 'A new timestep has just come in!'
        self.timestep = timestep.data
        print 'timestep is now '
        print self.timestep
        return

    def newObservationCallback(self, observation):
        """ Add new observation to observation queue """
        self.observations.append(observation)
        self.checkNewObservations()


    def checkNewObservations(self):
        """ Checks if new observations are valid
        observation type and then appends them."""
        if len(self.observations) > 0:
            unparsedObs = self.observations.pop()
            parsedObs='holds('

            if not unparsedObs.negation:
                parsedObs= '-' + parsedObs

            parsedObs += unparsedObs.fluent + '(' + unparsedObs.argument1 + ',' + unparsedObs.argument2 + '),' + str(unparsedObs.timestep) + ').'
            print parsedObs

        else:
            return

    def getExpectedConfig(self, timestep):

        state = []
        fpath_answer = os.path.join(os.path.dirname(__file__),'asp.answer')

        # for line in open(fpath_answer):
        anslist = []
        with open(fpath_answer) as infile:
            raw = infile.read()
            raw = re.findall("\{(.*?)\}", raw)[0]  # The answer set is inside the squiggly brackets {}
            # remove spaces between steps
            anslist = raw.split(' ')
            pattern = 'holds\(on\(block.*?,s.*?\),'+ str(int(timestep) +1) + '\)'
            # print 'match'
            matches=[]
            for thing in anslist:
                for match in re.findall(pattern, thing):
                    # print match
                    temp = re.findall("\((.*)\)", match)
                    temp = temp[0].split(',')
                    # isolate block id
                    temp[0] =  int(re.search('\(block(.*)', temp[0]).group(1))
                    # isolate surface
                    temp[1] = re.search("(.*)\)", temp[1]).group(1)
                    # replace with actual block or table
                    temp[1] = self.querySurface(temp[1])
                    temp[2] = int(temp[2])
                    # print temp
                    if 'tab' in temp[1] :
                        temp[1] = -1
                    else:
                        temp[1] =  int(re.search('block(.*)', temp[1]).group(1))
                    # print temp
                    # now first element is block id, second element is the block that
                    # the first block is on, the third element is the timestep
                    matches.append(temp)

        matches.sort(key=lambda x: int(x[0]))
        # print matches
        block_id = 0
        temp = []
        for block in matches:
            if (block[0] != block_id):
                # expected block id is in robot's hand
                temp.append(-2)
                block_id += 1
            temp.append(block[1])
            block_id += 1

        if (-2 not in temp) and (self.tableSurface>len(temp)):
            temp.append(-2)
        # print temp

        return Configuration(temp)






if __name__ == "__main__":

    sparc_path = sys.argv[1]
    asp_path = sys.argv[2]
    kb = ASPInterface(sparc_path, asp_path)

    rospy.Subscriber('/controller/timestep', Int16, kb.newTimestepCallback)
    # rospy.Subscriber('/controller/goal', Goal, kb.newGoalCallback)
    rospy.Subscriber('/controller/observations', Observation, kb.newObservationCallback)

    print "Ready to service queries"

    rospy.spin()
