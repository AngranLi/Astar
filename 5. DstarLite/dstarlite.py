'''
/* D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) */
/* Note: this version of D* Lite is optimized for grids                  */
/* It assumes, for example, that no cell can be a successor of itself.   */
'''
import math
import random
import rospy
import inc
import initEnv
import prioQ
import init
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

def heuristic(cell):
    global maze
    global mazegoal
    return abs(cell.x-mazegoal.x) + abs(cell.y-mazegoal.y) + abs(cell.z-mazegoal.z)

def publishactualmaze():
    global maze
    global mazestart
    global mazegoal

    print 'Publishing actual maze...'

    sourcePoint  	= init.initPointMarkers()
    goalPoint		= init.initPointMarkers()
    sourcePoint.color.g	= 1.0
    sourcePoint.color.a	= 1.0
    goalPoint.color.r	= 1.0
    goalPoint.color.a	= 1.0
    sourcePoint.id	= 0
    goalPoint.id	= 1

    actualObstPoint = init.initObstMarkers()

    # sourcePoint
    tempPoint = Point()
    tempPoint.x = mazegoal.x
    tempPoint.y = mazegoal.y
    tempPoint.z = mazegoal.z
    sourcePoint.points.append(tempPoint)
    sourcePoint.pose.position.x = mazegoal.x
    sourcePoint.pose.position.y = mazegoal.y
    sourcePoint.pose.position.z = mazegoal.z

    # goalPoint
    tempPoint = Point()
    tempPoint.x = mazestart.x
    tempPoint.y = mazestart.y
    tempPoint.z = mazestart.z
    goalPoint.points.append(tempPoint)
    goalPoint.pose.position.x = mazestart.x
    goalPoint.pose.position.y = mazestart.y
    goalPoint.pose.position.z = mazestart.z

    for x in range(inc.MAZELENGTH):
        for y in range(inc.MAZEWIDTH):
            for z in range(inc.MAZEHEIGHT):
                if maze[x][y][z].obstacle:
                    tempPoint = Point()
                    tempPoint.x = x
                    tempPoint.y = y
                    tempPoint.z = z
                    actualObstPoint.points.append(tempPoint)

	pointPub.publish(sourcePoint)
	pointPub.publish(goalPoint)
	obstPub.publish(actualObstPoint)


def publishknownmaze():
    global maze
    global mazestart
    global mazegoal

    print 'Publishing known maze...'

    pathPoint = init.initPathMarkers()
    currentPoint = init.initPointMarkers()
    currentPoint.color.r = 1.0
    currentPoint.color.g = 1.0
    currentPoint.id	= 3
    knownObstPoint = init.initObstMarkers()
    knownObstPoint.color.r = 0.8
    knownObstPoint.color.g = 0.4
    knownObstPoint.id = 31

    for x in range(inc.MAZELENGTH):
        for y in range(inc.MAZEWIDTH):
            for z in range(inc.MAZEHEIGHT):
                for d in range(inc.DIRECTIONS):
                    if maze[x][y][z].move[d]:
                        break
                    elif maze[x][y][z].obstacle:
                    	tempPoint = Point()
                        tempPoint.x = x
                        tempPoint.y = y
                        tempPoint.z = z
                        knownObstPoint.points.append(tempPoint)


    tmpcell = mazegoal
    while tmpcell != mazestart:
    	tempPoint = Point()
    	tempPoint.x = tmpcell.x
        tempPoint.y = tmpcell.y
        tempPoint.z = tmpcell.z
        pathPoint.points.append(tempPoint)
        tmpcell = tmpcell.searchtree

    tempPoint = Point()
    tempPoint.x = mazegoal.x
    tempPoint.y = mazegoal.y
    tempPoint.z = mazegoal.z
    currentPoint.points.append(tempPoint)
    currentPoint.pose.position.x = mazegoal.x
    currentPoint.pose.position.y = mazegoal.y
    currentPoint.pose.position.z = mazegoal.z

    pathPub.publish(pathPoint)
    pointPub.publish(currentPoint)
    obstPub.publish(knownObstPoint)


def initialize():
    global mazestart
    global mazegoal
    global keymodifier

    initEnv.mazeiteration = initEnv.mazeiteration + 1
    keymodifier = 0
    mazestart.g = inc.LARGE
    mazestart.rhs = 0
    if inc.TIEBREAKING:
        prioQ.emptyheap(3)
        mazestart.key[0] = heuristic(mazestart)
        mazestart.key[1] = heuristic(mazestart) + 1
        mazestart.key[2] = heuristic(mazestart)
    else:
        prioQ.emptyheap(2)
        mazestart.key[0] = heuristic(mazestart)
        mazestart.key[1] = 0

    mazestart.searchtree = None
    mazestart.generated = initEnv.mazeiteration
    prioQ.insertheap(mazestart)
    mazegoal.g = inc.LARGE
    mazegoal.rhs = inc.LARGE
    mazegoal.searchtree = None
    mazegoal.generated = initEnv.mazeiteration

#if RANDOMIZESUCCS:

def swappermutations(n):
    global permute
    global permutation
    global permutations

    if n:
    	for i in range(n+1):
    	    swappermutations(n-1)
    	    if n % 2:
                swap = permute[n]
                permute[n] = permute[i]
                permute[i] = swap
    	    else:
                swap = permute[n]
                permute[n] = permute[0]
                permute[0] = swap
    else:
        for i in range(inc.DIRECTIONS):
            permutation[i][permutations] = permute[i]
        permutations = permutations + 1


def createpermutations():
    global permute
    global permutation
    global permutations

    # int permute[DIRECTIONS]
    permute = []
    for i in range(inc.DIRECTIONS):
        permute.append(0)
    # int permutations
    permutations = 2
    for i in range(3,inc.DIRECTIONS+1):
        permutations *= i
    # int* permutation[DIRECTIONS]
    permutation = []
    for i in range(inc.DIRECTIONS):
        permute[i] = i
        row = []
        for m in range(permutations):
            row.append(0)
        permutation.append(row)


    permutations = 0
    swappermutations(inc.DIRECTIONS-1)

#endif

def initializecell(thiscell):
    if thiscell.generated != initEnv.mazeiteration:
        thiscell.g = inc.LARGE
        thiscell.rhs = inc.LARGE
        thiscell.searchtree = None
        thiscell.generated = initEnv.mazeiteration


def updatecell(thiscell):
    global keymodifier

    if thiscell.g < thiscell.rhs:
        if inc.TIEBREAKING:
            thiscell.key[0] = thiscell.g + heuristic(thiscell) + keymodifier
            thiscell.key[1] = thiscell.g + heuristic(thiscell) + keymodifier
            thiscell.key[2] = thiscell.g
        else:
            thiscell.key[0] = thiscell.g + heuristic(thiscell) + keymodifier
            thiscell.key[1] = thiscell.g
        prioQ.insertheap(thiscell)

    elif thiscell.g > thiscell.rhs:
        if inc.TIEBREAKING:
            thiscell.key[0] = thiscell.rhs + heuristic(thiscell) + keymodifier
            thiscell.key[1] = thiscell.rhs + heuristic(thiscell) + keymodifier + 1
            thiscell.key[2] = heuristic(thiscell) + keymodifier
        else:
            thiscell.key[0] = thiscell.rhs + heuristic(thiscell) + keymodifier
            thiscell.key[1] = thiscell.rhs
        prioQ.insertheap(thiscell)

    else:
	    prioQ.deleteheap(thiscell)


def updatekey(thiscell):
    global keymodifier

    if thiscell.g < thiscell.rhs:
        if inc.TIEBREAKING:
            thiscell.key[0] = thiscell.g + heuristic(thiscell) + keymodifier
            thiscell.key[1] = thiscell.g + heuristic(thiscell) + keymodifier
            thiscell.key[2] = thiscell.g
        else:
            thiscell.key[0] = thiscell.g + heuristic(thiscell) + keymodifier
    	    thiscell.key[1] = thiscell.g

    else:
        if inc.TIEBREAKING:
            thiscell.key[0] = thiscell.rhs + heuristic(thiscell) + keymodifier
            thiscell.key[1] = thiscell.rhs + heuristic(thiscell) + keymodifier + 1
            thiscell.key[2] = heuristic(thiscell) + keymodifier
        else:
            thiscell.key[0] = thiscell.rhs + heuristic(thiscell) + keymodifier
            thiscell.key[1] = thiscell.rhs


def updaterhs(thiscell):
    global permute
    global permutation
    global permutations

    thiscell.rhs = inc.LARGE
    thiscell.searchtree = None

    #ifdef RANDOMIZESUCCS
    dcase = random.randint(0, permutations-1)
    for dtemp in range(inc.DIRECTIONS):
        d = permutation[dtemp][dcase]
        #else
        '''
        for d in range(inc.DIRECTIONS):
        '''
        #endif
        if thiscell.move[d] and thiscell.move[d].generated == initEnv.mazeiteration and thiscell.rhs > thiscell.move[d].g + 1:
            thiscell.rhs = thiscell.move[d].g + 1
            thiscell.searchtree = thiscell.move[d]

    updatecell(thiscell)


def computeshortestpath():
    global maze
    global mazestart
    global mazegoal
    global keymodifier
    global permute
    global permutation
    global permutations
    goaltmpcell = initEnv.Cell()
    oldtmpcell  = initEnv.Cell()

    # print 'computing shortest path...'
    if inc.TIEBREAKING:
        if mazegoal.g < mazegoal.rhs:
            goaltmpcell.key[0] = mazegoal.g + keymodifier
            goaltmpcell.key[1] = mazegoal.g + keymodifier
            goaltmpcell.key[2] = mazegoal.g
        else:
            goaltmpcell.key[0] = mazegoal.rhs + keymodifier
            goaltmpcell.key[1] = mazegoal.rhs + keymodifier + 1
            goaltmpcell.key[2] = keymodifier
    else:
        if mazegoal.g < mazegoal.rhs:
            goaltmpcell.key[0] = mazegoal.g + keymodifier
            goaltmpcell.key[1] = mazegoal.g
        else:
            goaltmpcell.key[0] = mazegoal.rhs + keymodifier
            goaltmpcell.key[1] = mazegoal.rhs

    while prioQ.topheap() and (mazegoal.rhs > mazegoal.g or prioQ.keyless(prioQ.topheap(), goaltmpcell)):
        tmpcell1 = prioQ.topheap()
        oldtmpcell.key[0] = tmpcell1.key[0]
        oldtmpcell.key[1] = tmpcell1.key[1]

        if inc.TIEBREAKING:
            oldtmpcell.key[2] = tmpcell1.key[2]

        updatekey(tmpcell1)
        if prioQ.keyless(oldtmpcell, tmpcell1):
            # print 'in the if branch'
            updatecell(tmpcell1)
        elif tmpcell1.g > tmpcell1.rhs:
            # print 'in the elif branch'
            # print 'mazegoal.rhs(elif start): ', mazegoal.rhs
            tmpcell1.g = tmpcell1.rhs
            prioQ.deleteheap(tmpcell1)

            #ifdef RANDOMIZESUCCS
            dcase = random.randint(0, permutations-1)
            for dtemp in range(inc.DIRECTIONS):
                d = permutation[dtemp][dcase]
                '''
                #else
                for d in range(inc.DIRECTIONS):
                '''
                #endif
                if tmpcell1.move[d]:
                    tmpcell2 = tmpcell1.move[d]
                    initializecell(tmpcell2)
                    if tmpcell2 != mazestart and tmpcell2.rhs > tmpcell1.g + 1:
                        tmpcell2.rhs = tmpcell1.g + 1
                        # print 'tmpcell2 searchtree assigned!'
                        tmpcell2.searchtree = tmpcell1
                        updatecell(tmpcell2)
            # print 'tmpcell2.rhs(elif end): ', tmpcell2.rhs
            # print 'mazegoal.rhs(elif end): ', mazegoal.rhs
        else:
            # print 'in the else branch'
            tmpcell1.g = inc.LARGE
            updatecell(tmpcell1)

            #ifdef RANDOMIZESUCCS
            dcase = random.randint(0, permutations-1)
            for dtemp in range(inc.DIRECTIONS):
                d = permutation[dtemp][dcase]
                '''
                #else
                for d in range(inc.DIRECTIONS):
                '''
                #endif
                if tmpcell1.move[d]:
                    tmpcell2 = tmpcell1.move[d]
                    initializecell(tmpcell2)
                    if tmpcell2 != mazestart and tmpcell2.searchtree == tmpcell1:
                        tmpcell2 = updaterhs(tmpcell2)

        if inc.TIEBREAKING:
            if mazegoal.g < mazegoal.rhs:
                goaltmpcell.key[0] = mazegoal.g + keymodifier
                goaltmpcell.key[1] = mazegoal.g + keymodifier
                goaltmpcell.key[2] = mazegoal.g
            else:
                goaltmpcell.key[0] = mazegoal.rhs + keymodifier
                goaltmpcell.key[1] = mazegoal.rhs + keymodifier + 1
                goaltmpcell.key[2] = keymodifier
        else:
            if mazegoal.g < mazegoal.rhs:
                goaltmpcell.key[0] = mazegoal.g + keymodifier
                goaltmpcell.key[1] = mazegoal.g
            else:
                goaltmpcell.key[0] = mazegoal.rhs + keymodifier
                goaltmpcell.key[1] = mazegoal.rhs

    return (mazegoal.rhs == inc.LARGE)



def updatemaze(robot):
    global mazestart
    global permute
    global permutation
    global permutations

    #ifdef RANDOMIZESUCCS
    dcase = random.randint(0, permutations-1)
    for dtemp in range(inc.DIRECTIONS):
        d1 = permutation[dtemp][dcase]
        #else
        # for d1 in range(inc.DIRECTIONS):
        #endif
        if robot.move[d1] and robot.move[d1].obstacle:
            tmpcell = robot.move[d1]
            initializecell(tmpcell)
            for d2 in range(inc.DIRECTIONS):
                if tmpcell.move[d2]:
                    tmpcell.move[d2] = None
                    tmpcell.succ[d2].move[inc.reverse[d2]] = None
                    initializecell(tmpcell.succ[d2])
                    if tmpcell.succ[d2] != mazestart and tmpcell.succ[d2].searchtree == tmpcell:
                        tmpcell.succ[d2] = updaterhs(tmpcell.succ[d2])
            if tmpcell != mazestart:
                tmpcell.rhs = inc.LARGE
                updatecell(tmpcell)


################################################################################
################################################################################

rospy.init_node('dstar_node', anonymous=True) # rosnode name
(pathPub, pointPub, boundPub, obstPub) = init.initPublishers() # initialize publishers

#ifdef RANDOMIZESUCCS
createpermutations()
#endif

keymodifier = 0

for k in range(inc.RUNS):
    print '====================================================================='
    print 'maze', k

    initEnv.newrandommaze()
    # initEnv.newdfsmaze(inc.WALLSTOREMOVE)

    maze 		= initEnv.maze
    mazestart   = initEnv.mazestart
    mazegoal    = initEnv.mazegoal

    # print 'Start point: '
    # print 'Destination: '
    publishactualmaze()

    initialize()
    lastcell = mazegoal
    while mazestart != mazegoal:
        if computeshortestpath():
            print 'breaaaaak'
            break

        publishknownmaze()
        rospy.sleep(0.1)

        mazegoal.trace = None
        while True:
            print 'pos. of mazegoal 1: ', (mazegoal.x, mazegoal.y, mazegoal.z)
            mazegoal.searchtree.trace = mazegoal
            mazegoal = maze[mazegoal.searchtree.x][mazegoal.searchtree.y][mazegoal.searchtree.z] # mazegoal.searchtree # 
            # print 'pos. of mazegoal 2: ', (mazegoal.x, mazegoal.y, mazegoal.z)
            if mazestart == mazegoal or mazegoal.searchtree.obstacle:
                break

        if mazestart != mazegoal:
            keymodifier = keymodifier + heuristic(lastcell)
            lastcell = mazegoal

            tmpcell = mazegoal
            while tmpcell:
                updatemaze(tmpcell)
                tmpcell = tmpcell.trace

    print 'Finished.'
    rospy.sleep(3)
