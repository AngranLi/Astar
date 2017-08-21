'''
/* D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) */
/* Note: this version of D* Lite is optimized for grids                  */
/* It assumes, for example, that no cell can be a successor of itself.   */
'''
import math
import random
import timeit
import rospy
import inc
import initEnv
import prioQ
import init
from geometry_msgs.msg import PointStamped, Point, PoseStamped
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
    global pathPoint

    print 'Publishing known maze...'

    pathPoint = init.initPathMarkers()
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
    tempPoint.x = mazestart.x
    tempPoint.y = mazestart.y
    tempPoint.z = mazestart.z
    pathPoint.points.append(tempPoint)

    pathPub.publish(pathPoint)
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


def callback_current(data):  # data contains the current point
    # rospy.loginfo((data.pose.position.x, data.pose.position.y, data.pose.position.z))
    global current_position
    global currentPoint
    current_position = (data.pose.position.x, data.pose.position.y, data.pose.position.z)

    temptuple = tuple(init.gridalize(current_position, scale))
    tempPoint = Point()
    tempPoint.x = temptuple[0]
    tempPoint.y = temptuple[1]
    tempPoint.z = temptuple[2]
    del currentPoint.points[:]
    currentPoint.points.append(tempPoint)
    currentPoint.pose.position.x = temptuple[0]
    currentPoint.pose.position.y = temptuple[1]
    currentPoint.pose.position.z = temptuple[2]
    pointPub.publish(currentPoint)

    # print 'currentPoint pos. 3', currentPoint.points[0].x, currentPoint.points[0].y, currentPoint.points[0].z

    callback_current_flg = True

def callback_obst_UAV1(centrePos):
    centrePoint = (int(centrePos.pose.position.x*scale), int(centrePos.pose.position.y*scale), int(centrePos.pose.position.z*scale))
    tempobst = initEnv.Obstacle('obst_UAV1', centrePoint)
    initEnv.updateObst(tempobst)

    callback_obst_UAV1_flg = True


def callback_obst_UGV1(centrePos):
    centrePoint = (int(centrePos.pose.position.x*scale), int(centrePos.pose.position.y*scale), int(centrePos.pose.position.z*scale))
    tempobst = initEnv.Obstacle('obst_UGV1', centrePoint)
    initEnv.updateObst(tempobst)

    callback_obst_UGV1_flg = True


def callback_obst_person1(centrePos):
    centrePoint = (int(centrePos.pose.position.x*scale), int(centrePos.pose.position.y*scale), int(centrePos.pose.position.z*scale))
    tempobst = initEnv.Obstacle('obst_person1', centrePoint)
    initEnv.updateObst(tempobst)

    callback_obst_person1_flg = True

################################################################################
################################################################################
################################################################################

''' Set original environment '''
mapBound_metre = (5, 5, 3)      # 3D boundary of the operating environment
scale = 10

startPoint  = (0, 0, 0)
endPoint    = (0, 0, 0)
current_position  = (1, 1, 1)
target_position   = (4.5, 4.0, 2.0)

''' Initialize ROS node and publishers '''
rospy.init_node('dstar_node', anonymous=True) # rosnode name
(pathPub, pointPub, boundPub, obstPub) = init.initPublishers() # initialize publishers
rospy.sleep(0.3) # it takes time to initialize publishers
rate = rospy.Rate(10) # loop runs at x Hertz

''' Set original value of flags '''
# gotPath = False
# pathBlocked = True
callback_obst_UAV1_flg = True
callback_obst_UGV1_flg = True
callback_obst_person1_flg = True
callback_current_flg = True
# callback_target_flg = True

''' Performance measurement '''
len_of_path = 0
execution_time = 0
vertex_expension = 0

''' Intialize markers '''
currentPoint = init.initPointMarkers()
currentPoint.color.r = 1.0
currentPoint.color.g = 1.0
currentPoint.id	= 3
temptuple = tuple(init.gridalize(current_position, scale))
tempPoint = Point()
tempPoint.x = temptuple[0]
tempPoint.y = temptuple[1]
tempPoint.z = temptuple[2]
currentPoint.points.append(tempPoint)
currentPoint.pose.position.x = temptuple[0]
currentPoint.pose.position.y = temptuple[1]
currentPoint.pose.position.z = temptuple[2]

pathPoint = init.initPathMarkers()
knownObstPoint = init.initObstMarkers()
knownObstPoint.color.r = 0.8
knownObstPoint.color.g = 0.4
knownObstPoint.id = 31


#ifdef RANDOMIZESUCCS
createpermutations()
#endif

keymodifier = 0

print '====================================================================='

initEnv.newEmptyEnv()
# initEnv.newdfsmaze(inc.WALLSTOREMOVE)

# receive obstacle position
while callback_obst_UAV1_flg:
    obstSub = rospy.Subscriber('UAV_2/pose', PoseStamped, callback_obst_UAV1)
    callback_obst_UAV1_flg = False
while callback_obst_UGV1_flg:
    obstSub = rospy.Subscriber('/UAV_3/pose', PoseStamped, callback_obst_UGV1)
    callback_obst_UGV1_flg = False
while callback_obst_person1_flg:
    obstSub = rospy.Subscriber('/UAV_4/pose', PoseStamped, callback_obst_person1)
    callback_obst_person1_flg = False

# receive current position of UAV
while callback_current_flg:
    ppSub   = rospy.Subscriber('/UAV_1/pose', PoseStamped, callback_current)
    callback_current_flg = False
rospy.sleep(0.3)

maze 		= initEnv.maze
mazestart   = initEnv.mazestart
mazegoal    = maze[currentPoint.points[0].x][currentPoint.points[0].y][currentPoint.points[0].z] # initEnv.mazegoal

publishactualmaze()
# rospy.sleep(0.1)

initialize()
lastcell = mazegoal

while mazestart != mazegoal:
    if computeshortestpath():
        rospy.logfatal('breaaaaak')
        break

    publishknownmaze()
    # rospy.sleep(0.1)

    mazegoal.trace = None
    while True:
        publishactualmaze()
        # rospy.sleep(0.1)

        dist_min = 200
        closestPoint_id = 0
        try:
            for i in range(len(pathPoint.points)):
                dist = abs(pathPoint.points[i].x-currentPoint.points[0].x) + abs(pathPoint.points[i].y-currentPoint.points[0].y) + abs(pathPoint.points[i].z-currentPoint.points[0].z)
                if dist < dist_min:
                    dist_min = dist
                    closestPoint_id = i
        except IndexError:
            print 'i is ', i
            print 'len(pathPoint.points): ', len(pathPoint.points)
        closestPoint = (pathPoint.points[closestPoint_id].x, pathPoint.points[closestPoint_id].y, pathPoint.points[closestPoint_id].z)
        mazegoal.searchtree.trace = mazegoal
        mazegoal = maze[closestPoint[0]][closestPoint[1]][closestPoint[2]] # mazegoal.searchtree #
        print 'pos. of mazestart: ', (mazestart.x, mazestart.y, mazestart.z)
        print 'pos. of mazegoal:  ', (mazegoal.x, mazegoal.y, mazegoal.z)
        publishknownmaze()
        # rospy.sleep(0.1)
        if mazestart == mazegoal or mazegoal.searchtree.obstacle:
            rospy.logwarn('detected obstacle!')
            break

    if mazestart != mazegoal:
        keymodifier = keymodifier + heuristic(lastcell)
        lastcell = mazegoal

        tmpcell = mazegoal
        while tmpcell:
            updatemaze(tmpcell)
            tmpcell = tmpcell.trace

print 'Finished.'
print 'Heap percolation: ', heap_percolation
