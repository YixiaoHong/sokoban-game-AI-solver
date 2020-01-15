#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
from search import *  # for search engines
from sokoban import SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems


def sokoban_goal_state(state):
    '''
    @return: Whether all boxes are stored.
    '''
    for box in state.boxes:
        if box not in state.storage:
            return False
    return True


def heur_manhattan_distance(state):  # Sokoban State contains: action, gval, parent, width, height, robots, boxes, storage, obstacles
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.

    totalManhattanDis = 0  # Default value of Total Manhattan Distance for all boxes
    boxManhattanDis = float('inf')  # Default value of Manhattan Distance for each box

    for i in state.boxes:  # Consider single box i
        for j in state.storage:  # Consider box i to storage j
            tempBoxManhattanDis = abs(i[0] - j[0]) + abs(i[1] - j[1])  # Calculate Distance form box i to storage j
            if tempBoxManhattanDis <= boxManhattanDis:  # Compair the smallest Distance for single box so far
                boxManhattanDis = tempBoxManhattanDis
        totalManhattanDis = totalManhattanDis + boxManhattanDis  # Add the smallest Distance to Total Manhattan Distance
        boxManhattanDis = float('inf')  # Reset Default value of Manhattan Distance for single box

    return totalManhattanDis


# SOKOBAN HEURISTICS
def trivial_heuristic(state):
    '''trivial admissible sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
    count = 0
    for box in state.boxes:
        if box not in state.storage:
            count += 1
    return count


####################################################################
##########    Function for heur_alternate    #######################
####################################################################

# Test OK
def left1(position, state):  # Get the Cordinate of a position left to the input
    if position[0] > 0:
        leftPos = (position[0] - 1, position[1])
        return leftPos
    else:
        return False

# Test OK
def right1(position, state):  # Get the Cordinate of a position right to the input
    if position[0] < state.width - 1:
        rightPos = (position[0] + 1, position[1])
        return rightPos
    else:
        return False

# Test OK
def up1(position, state):  # Get the Cordinate of a position up to the input
    if position[1] > 0:
        upPos = (position[0], position[1] - 1)
        return upPos
    else:
        return False

# Test OK
def down1(position, state):  # Get the Cordinate of a position down to the input
    if position[1] < state.height - 1:
        downPos = (position[0], position[1] + 1)
        return downPos
    else:
        return False


#################################################################
###############   Check Dead Lock Case    ####################
###############################################################

# Test OK
def checkCornerLock(state):  # Check whether there is cornerlock box in a given case

    caseCornerLocked = False

    for box in state.boxes:
        # print("Checking box:", box)
        if box in state.storage:
            # print("this box in target position !")
            continue
        else:
            L = 0
            U = 0
            R = 0
            D = 0
            if ((left1(box, state) in state.obstacles) or left1(box, state) == False):
                L = 1
            if ((right1(box, state) in state.obstacles) or right1(box, state) == False):
                R = 1
            if ((up1(box, state) in state.obstacles) or up1(box, state) == False):
                U = 1
            if ((down1(box, state) in state.obstacles) or down1(box, state) == False):
                D = 1
            # print("L", L, "U", U, "R", R, "D", D)

        if ((L == U == 1) or (U == R == 1) or (R == D == 1) or (D == L == 1)):
            # print("Corner Lock!")
            caseCornerLocked = True

    return caseCornerLocked

# Test OK
def checkBorderLock(state):  # Check whether there is a box near border but no avaliable storage left for it on this side


    # print("not solved storage:", avaliableStorage)

    caseBorderLocked = False

    for box in state.boxes:
        #print("Checking box:", box)
        avaliableStorage = set()
        boxAtBorder = set()
        if left1(box, state) == False:
            #print("Left Border")
            for j in range(state.height):
                if (0,j) in state.storage:
                    avaliableStorage.add((0,j))
                if (0,j)  in state.boxes:
                    boxAtBorder.add((0,j))
            #print("Left Box:", boxAtBorder, "avaliavle:", avaliableStorage)
            if len(boxAtBorder)>len(avaliableStorage):
                #print("Left Box:",boxAtBorder,"avaliavle:",avaliableStorage)
                return True

        avaliableStorage = set()
        boxAtBorder = set()
        if up1(box, state) == False:
            #print("Up Border")
            for i in range(state.width):
                if (i,0) in state.storage:
                    avaliableStorage.add((i,0))
                if (i,0) in state.boxes:
                    boxAtBorder.add((i,0))
            if len(boxAtBorder) > len(avaliableStorage):
                #print("Up Box:", boxAtBorder, "avaliavle:", avaliableStorage)
                return True

        avaliableStorage = set()
        boxAtBorder = set()
        if right1(box, state) == False:
            #print("Right Border")
            for j in range(state.height):
                if (state.width-1, j) in state.storage:
                    avaliableStorage.add((state.width-1, j))
                if (state.width-1, j) in state.boxes:
                    boxAtBorder.add((state.width-1, j))
            if len(boxAtBorder) > len(avaliableStorage):
                #print("Right Box:", boxAtBorder, "avaliavle:", avaliableStorage)
                return True

        avaliableStorage = set()
        boxAtBorder = set()
        if down1(box, state) == False:
            #print("Down Border")
            for i in range(state.width):
                if (i,state.height-1) in state.storage:
                    avaliableStorage.add((i, state.height-1))
                if (i,state.height-1) in state.boxes:
                    boxAtBorder.add((i, state.height-1))
            if len(boxAtBorder) > len(avaliableStorage):
                #print("Box:", boxAtBorder, "avaliavle:", avaliableStorage)
                return True

    return caseBorderLocked

# Test OK
def check2x2Locked(state):  # Check if box locked in a 2x2 space

    searchSpace = []
    for i in range(0, state.width):
        r = []
        for j in range(0, state.height):
            r = r + [' ']
        searchSpace += [r]

    for wall in state.obstacles:
        searchSpace[wall[0]][wall[1]] = '#'
    for box in state.boxes:
        if box in state.storage:
            searchSpace[box[0]][box[1]] = '*'
        else:
            searchSpace[box[0]][box[1]] = '$'
    # print (searchSpace)

    for i in range(0, state.width):
        searchSpace[i] = ['#'] + searchSpace[i]
        searchSpace[i] = searchSpace[i] + ['#']
    searchSpace = ['#' * (state.height + 2)] + searchSpace
    searchSpace = searchSpace + ['#' * (state.height + 2)]

    # searchSpace Created

    list4 = []
    lockedBoxIndex = set()

    for i in range(len(searchSpace) - 1):
        for j in range(len(searchSpace[0]) - 1):
            # print(i,j)
            list4.append(searchSpace[i][j])
            list4.append(searchSpace[i + 1][j])
            list4.append(searchSpace[i][j + 1])
            list4.append(searchSpace[i + 1][j + 1])
            # print(list4)
            if ((' ' not in list4) and ('$' in list4)):
                index = 0
                for n in list4:
                    if n == '$' or n == '*':
                        if index == 0:
                            lockedBoxIndex.add((i - 1, j - 1))
                        elif index == 1:
                            lockedBoxIndex.add((i, j - 1))
                        elif index == 2:
                            lockedBoxIndex.add((i - 1, j))
                        elif index == 3:
                            lockedBoxIndex.add((i, j))
                    index = index + 1

            list4 = []

    return lockedBoxIndex

#Test OK
def checkSpecialCornerLock(state):  # Check whether there is cornerlock box in a given case

    caseCornerLocked = False

    #check UL corner
    if (1,0) in state.boxes:
        if(0,1) in state.boxes:
            if (((1,0) not in state.storage) or ((0,1) not in state.storage)) and ((0,0) not in state.robots):
                #print("Special Corner Lock!")
                return True
    #check UR corner
    if (state.width-2,0) in state.boxes:
        if(state.width-1,1) in state.boxes:
            if (((state.width-2,0) not in state.storage) or ((state.width-1,1) not in state.storage)) and ((state.width-1,0) not in state.robots):
                #print("Special Corner Lock!")
                return True
    #check DL corner
    if (0,state.height-2) in state.boxes:
        if(1,state.height-1) in state.boxes:
            if (((0,state.height-2) not in state.storage) or ((1,state.height-1) not in state.storage)) and ((0,state.height-1) not in state.robots):
                #print("Special Corner Lock!")
                return True
    #check DR corner
    if (state.width-2,state.height-1) in state.boxes:
        if(state.width-1,state.height-2) in state.boxes:
            if (((state.width-2,state.height-1) not in state.storage) or ((state.width-1,state.height-2) not in state.storage)) and ((state.width-1,state.height -1) not in state.robots):
                #print("Special Corner Lock!")
                return True

    return caseCornerLocked


#################################################################
###############   Robot H Function    ####################
###############################################################

#Test OK
def robotHval(state):
    totalRobotManhattanDis = 0
    boxNearRobot = set()
    robotManhattanDis = 99999

    for i in state.robots:
        #print("checking robot,", i)
        for j in state.boxes:
            if state.boxes in state.storage:
                continue
            else:
                temprobotManhattanDis = abs(i[0] - j[0]) + abs(i[1] - j[1])
                if temprobotManhattanDis <= robotManhattanDis:
                    robotManhattanDis = temprobotManhattanDis

        totalRobotManhattanDis = totalRobotManhattanDis + robotManhattanDis
        robotManhattanDis = 99999

    for i in state.boxes:
        #print("check box:",i)
        if left1(i, state):
            if left1(i, state) in state.robots:
                #print("L robot")
                boxNearRobot.add(i)
                continue
        if right1(i, state):
            if right1(i, state) in state.robots:
                #print("R robot")
                boxNearRobot.add(i)
                continue
        if up1(i, state):
            if up1(i, state) in state.robots:
                #print("U robot")
                boxNearRobot.add(i)
                continue
            if left1(up1(i, state), state):
                if left1(up1(i, state), state) in state.robots:
                    #print("UL robot")
                    boxNearRobot.add(i)
                    continue
            if right1(up1(i, state), state):
                if right1(up1(i, state), state) in state.robots:
                    #print("UR robot")
                    boxNearRobot.add(i)
                    continue
        if down1(i, state):
            if down1(i, state) in state.robots:
                #print("D robot")
                boxNearRobot.add(i)
                continue
            if left1(down1(i, state), state):
                if left1(down1(i, state), state) in state.robots:
                    #print("DL robot")
                    boxNearRobot.add(i)
                    continue
            if right1(down1(i, state), state):
                if right1(down1(i, state), state) in state.robots:
                    #print("DR robot")
                    boxNearRobot.add(i)
                    continue
    #print(boxNearRobot)
    boxNoRobot = len(state.boxes) - len(boxNearRobot)
    #print(boxNoRobot)
    totalRobotManhattanDis = totalRobotManhattanDis + boxNoRobot

    return totalRobotManhattanDis

def robotHvalForCornerCase(state):
    totalRobotManhattanDis = 0
    robotManhattanDis = 99999
    for i in state.robots:
        for j in state.boxes:
            if state.boxes in state.storage:
                continue
            else:
                temprobotManhattanDis = abs(i[0] - j[0]) + abs(i[1] - j[1])
                if temprobotManhattanDis <= robotManhattanDis:
                    robotManhattanDis = temprobotManhattanDis

        totalRobotManhattanDis = totalRobotManhattanDis + robotManhattanDis
        robotManhattanDis = 99999
    return totalRobotManhattanDis

#################################################################
###############   H_Val Function Type Check    ####################
###############################################################

#Test OK
def checkList(string):    #Check if a list is continues 1111 and then 00000, if yes return size of 1, if not return 9999
    #print(string) #[1,0,0,0,1,1,0,1,1,0,1,1,1]
    initLen = len(string)
    index = 0
    lastItem = 'high'

    for i in string:
        if lastItem == 'high':
            if i == 1:
                lastItem = 'high'
            else:
                lastItem = 'low'
            index = index +1
        else:
            cut = index -1

    string.reverse()
    #print(string)

    for i in range(cut):
        string.pop()

    #print(string)
    for i in string:
        if i == 1:
            return 99999

    finalLen = len(string)
    size = initLen - finalLen

    return size

#Test OK
def checkProblemType(state):    #Return Regular, UL, UR, DL, DR
    ####Checking UL
    UL = False
    UR = False
    DL = False
    DR = False

    if (0,0) in state.storage:
        #print("Checking 0,0")
        xList = []
        for i in range(0,state.width):
            if (i,0) in state.storage:
                xList.append(1)
            else:
                xList.append(0)
        #print(xList)
        xRange = checkList(xList)
        #print("xRange:",xRange)
        if xRange == 99999:
            #print("X not good")
            return 'Regular'

        yList = []
        for i in range(0, state.height):
            if (0,i) in state.storage:
                yList.append(1)
            else:
                yList.append(0)
        #print(yList)
        yRange = checkList(yList)
        #print("yRange:", yRange)
        if yRange == 99999:
            #print("Y not good")
            return 'Regular'

        storageInRange = set()
        for x in range(xRange):
            for y in range(yRange):
                if (x,y) in state.storage:
                    storageInRange.add((x,y))
        #print(storageInRange)

        totalStorage = set()
        for a in state.storage:
            totalStorage.add(a)

        #print(totalStorage)

        for check in totalStorage:
            if check not in storageInRange:
                return 'Regular'
        else:
            #Print("UL True")
            UL = True
            return 'UL'

    if (state.width-1,0) in state.storage:
        #print("Checking UR")
        xList = []
        for i in range(0,state.width):
            if (i,0) in state.storage:
                xList.append(1)
            else:
                xList.append(0)
        xList.reverse()
        #print(xList)
        xRange = checkList(xList)
        #print("xRange:",xRange)
        if xRange == 99999:
            #print("X not good")
            return 'Regular'

        yList = []
        for i in range(0, state.height):
            if (state.width-1,i) in state.storage:
                yList.append(1)
            else:
                yList.append(0)
        #print(yList)
        yRange = checkList(yList)

        #print("yRange:", yRange)
        if yRange == 99999:
            #print("Y not good")
            return 'Regular'

        storageInRange = set()
        for x in range(state.width-xRange,state.width):
            for y in range(yRange):
                if (x,y) in state.storage:
                    storageInRange.add((x,y))

        #print("UR, storage in range")
        #print(storageInRange)

        totalStorage = set()
        for a in state.storage:
            totalStorage.add(a)
        #print("total Storage")
        #print(totalStorage)

        for check in totalStorage:
            if check not in storageInRange:
                return 'Regular'
        else:
            #print("UR True")
            UR = True
            return 'UR'

    if (0, state.height-1) in state.storage:
        #print("Checking DL")
        xList = []
        for i in range(0,state.width):
            if (i,state.height-1) in state.storage:
                xList.append(1)
            else:
                xList.append(0)
        #print(xList)
        xRange = checkList(xList)
        #print("xRange:",xRange)
        if xRange == 99999:
            #print("X not good")
            return 'Regular'

        yList = []
        for i in range(0, state.height):
            if (0,i) in state.storage:
                yList.append(1)
            else:
                yList.append(0)
        yList.reverse()
        #print(yList)
        yRange = checkList(yList)
        #print("yRange:", yRange)
        if yRange == 99999:
            #print("Y not good")
            return 'Regular'

        storageInRange = set()
        for x in range(xRange):
            for y in range(state.height-yRange,state.height):
                if (x, y) in state.storage:
                    storageInRange.add((x, y))

        #print("DL, storage in range")
        #print(storageInRange)

        totalStorage = set()
        for a in state.storage:
            totalStorage.add(a)
        #print("total Storage")
        #print(totalStorage)

        for check in totalStorage:
            if check not in storageInRange:
                return 'Regular'
        else:
            #print("DL True")
            DL = True
            return 'DL'

    if (state.width-1, state.height-1) in state.storage:
        #Print("Checking DR")
        xList = []
        for i in range(0,state.width):
            if (i,state.height-1) in state.storage:
                xList.append(1)
            else:
                xList.append(0)
        xList.reverse()
        #Print(xList)
        xRange = checkList(xList)
        #Print("xRange:",xRange)
        if xRange == 99999:
            #Print("X not good")
            return 'Regular'

        yList = []
        for i in range(0, state.height):
            if (state.width - 1, i) in state.storage:
                yList.append(1)
            else:
                yList.append(0)
        yList.reverse()
        #Print(yList)
        yRange = checkList(yList)

        #Print("yRange:", yRange)
        if yRange == 99999:
            #Print("Y not good")
            return 'Regular'

        storageInRange = set()
        for x in range(state.width - xRange, state.width):
            for y in range(state.height-yRange,state.height):
                if (x, y) in state.storage:
                    storageInRange.add((x, y))

        #Print("DR, storage in range")
        #Print(storageInRange)

        totalStorage = set()
        for a in state.storage:
            totalStorage.add(a)
        #Print("total Storage")
        #Print(totalStorage)

        for check in totalStorage:
            if check not in storageInRange:
                return 'Regular'
        else:
            #Print("DR True")
            UR = True
            return 'DR'

    #Print("UL",UL,"UR",UR,"DL",DL,"DR",DR)
    return 'Regular'

#################################################################
###############   H     Val     Function    ####################
###############################################################

def regular_h_f(state):
    # Calculate Box Manhattan Dis:
    totalboxManhattanDis = 0
    boxManhattanDis = 99999  # Default value of Manhattan Distance for each box

    for i in state.boxes:  # Consider single box i
        for j in state.storage:  # Consider box i to storage j
            tempBoxManhattanDis = abs(i[0] - j[0]) + abs(i[1] - j[1])  # Calculate Distance form box i to storage j
            if tempBoxManhattanDis <= boxManhattanDis:  # Compair the smallest Distance for single box so far
                boxManhattanDis = tempBoxManhattanDis
        totalboxManhattanDis = totalboxManhattanDis + boxManhattanDis  # Add the smallest Distance to Total Manhattan Distance
        boxManhattanDis = 99999  # Default value of Manhattan Distance for single box

    #####Sum up final Manhattan Dis:
    h_val = totalboxManhattanDis + robotHval(state)

    return h_val

def corner_consentrated_storage_h_f(state,corner):

    if corner == 'UL':
        totalboxManhattanDis = 0
        for i in state.boxes:
            boxManhattanDis = abs(i[0] - 0) + abs(i[1] - 0)
            totalboxManhattanDis = totalboxManhattanDis + boxManhattanDis
        if (0,0) not in state.boxes:
            totalboxManhattanDis = totalboxManhattanDis + 1
        return totalboxManhattanDis

    elif corner == 'UR':
        totalboxManhattanDis = 0
        for i in state.boxes:
            boxManhattanDis = abs(i[0] - state.width+1) + abs(i[1] - 0)
            totalboxManhattanDis = totalboxManhattanDis + boxManhattanDis
        if (state.width-1,0) not in state.boxes:
            totalboxManhattanDis = totalboxManhattanDis + 1
        return totalboxManhattanDis

    elif corner == 'DL':
        totalboxManhattanDis = 0
        for i in state.boxes:
            boxManhattanDis = abs(i[0] - 0) + abs(i[1] - state.height+1)
            totalboxManhattanDis = totalboxManhattanDis + boxManhattanDis
        if (0,state.height-1) not in state.boxes:
            totalboxManhattanDis = totalboxManhattanDis + 1
        return totalboxManhattanDis

    elif corner == 'DR':
        totalboxManhattanDis = 0
        for i in state.boxes:
            boxManhattanDis = abs(i[0] - state.width+1) + abs(i[1] - state.height+1)
            totalboxManhattanDis = totalboxManhattanDis + boxManhattanDis
        if (state.width-1,state.height-1) not in state.boxes:
            totalboxManhattanDis = totalboxManhattanDis + 1
        return totalboxManhattanDis




def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.

    h_val = 0  # Default value of Total Manhattan Distance for all boxes

    # Check Dead Lock First:

    if checkCornerLock(state) == True:

        return 99999

    if checkBorderLock(state) == True:

        return 99999

    locked2x2 = check2x2Locked(state)
    if locked2x2 != set():
        for i in locked2x2:
            if i not in state.storage:
                return 99999

    if checkSpecialCornerLock(state) == True:
        return 99999

    checkProblemCase = checkProblemType(state)  #Check Which Type of Problem is given and assign different H_val calculate function

    if checkProblemCase == 'Regular':
        h_val = regular_h_f(state)        #Apply Regular H_Val Function

    if checkProblemCase == 'UL':
        h_val = corner_consentrated_storage_h_f(state, 'UL')  #Apply UpperLeft Concentrated Storage H_Val Calculation Function
    if checkProblemCase == 'UR':
        h_val = corner_consentrated_storage_h_f(state, 'UR')  #Apply UpperRight Concentrated Storage H_Val Calculation Function
    if checkProblemCase == 'DL':
        h_val = corner_consentrated_storage_h_f(state, 'DL')  #Apply DownLeft Concentrated Storage H_Val Calculation Function
    if checkProblemCase =='DR':
        h_val = corner_consentrated_storage_h_f(state, 'DR')  #Apply DownRight Concentrated Storage H_Val Calculation Function


    return h_val


###############################################################
###############################################################
###############################################################


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
    #IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    fValual = sN.gval + weight*sN.hval    #By Definition of F-value function for Anytime Weighted A Star Method
    return fValual

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
    #IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''

    wrapped_fval_function = (lambda sN: fval_function(sN, weight))                                                      #Set up function to get wrapped f_val
    bestSolutionSoFar = False                                                                                           # Initial Defult FeedBack Set to be False
    costBound = (99999, 99999, 99999)                                                                                   # Initial Defult costBound set to be infinite for gval, hval, g+h val
    searchEngine = SearchEngine('custom', 'full')                                                                       # Set Up Search Engine in best custom search type since we are using fval function and full cycle check
    searchEngine.init_search(initial_state, sokoban_goal_state, heur_fn,wrapped_fval_function)                          # Get ready for the search engine
    currentTime = os.times()[0]                                                                                         # Initialize timer
    stopTime = currentTime + timebound                                                                                  # Calculate the time the program has to stop
    initialWeight = weight                                                                                              # Store initial input weight
    iteration = 0                                                                                                       #counter to record search iteration
    startTime = os.times()[0]

    print("Doing iteration:", iteration, "weight", weight)
    while currentTime < stopTime:

        if iteration !=0:
            weight = 1 + (0.5 ** iteration) * (initialWeight - 1)                                                       #Decrease the amont of weight exceed 1 with 10% comparing to last iteration
            wrapped_fval_function = (lambda sN: fval_function(sN, weight))                                              #re-define wrapped f_val function and put in new weight
            searchEngine.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)                 #re-initialize search engine
            print("Doing iteration:", iteration, "weight", weight)

        solution = searchEngine.search(stopTime - currentTime, costBound)                                               # Apply Search Engine to get solution

        if solution != False:                                                                                           # If solution find before time limit
            if solution.gval < costBound[0]:                                                                            # Compair Current Solution g_val to so far best solution g_val
                bestSolutionSoFar = solution                                                                            # If better, replace so far best solution
                costBound = (99999, 99999, bestSolutionSoFar.gval)                              # replace so far best solution's gval and compair later g+h val with this g val for pruning
        elif solution == False:
            return bestSolutionSoFar
        currentTime = os.times()[0]                                                                                     # Update current time for next loop

        iteration = iteration + 1

    return bestSolutionSoFar

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
    #IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of anytime_gbfs'''

    searchEngine = SearchEngine('best_first','full')  # Set Up Search Engine in best first strategy search and full cycle check
    searchEngine.init_search(initial_state, sokoban_goal_state, heur_fn)  # Get ready for the search engine

    bestSolutionSoFar = False                                                       #Initial Defult FeedBack Set to be False
    costBound = (99999, 99999, 99999)                                               #Initial Defult costBound set to be infinite for gval, hval, g+h val

    currentTime = os.times()[0]                                                     #Initialize timer
    stopTime = currentTime + timebound                                              #Calculate the time the program has to stop

    while currentTime < stopTime:
        solution = searchEngine.search(stopTime-currentTime, costBound)             #Apply Search Engine to get solution

        if solution != False:                                                      #If solution find before time limit
            if solution.gval < costBound[0]:                                        #Compair Current Solution g_val to so far best solution g_val
                bestSolutionSoFar = solution                                        #If better, replace so far best solution
                costBound = (bestSolutionSoFar.gval, 99999, 99999)    #replace so far best solution's gval and compair later g val with this g val for pruning
        elif solution == False:
            return bestSolutionSoFar
        currentTime = os.times()[0]                                                 #Update current time for next loop

    return bestSolutionSoFar


#########################################
#####         For Test          #########
#########################################

