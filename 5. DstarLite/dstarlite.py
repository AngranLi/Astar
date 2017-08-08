'''
/* D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) */
/* Note: this version of D* Lite is optimized for grids                  */
/* It assumes, for example, that no cell can be a successor of itself.   */
'''
import random
import rospy
import inc
import initMaze
import initHeap

def heuristic(cell):
    global maze
    global mazegoal
    return abs(cell.y-mazegoal.y) + abs(cell.x-mazegoal.x)

def printactualmaze():
    global maze
    global mazestart
    global mazegoal

    print 'Actual maze: '

    for x in range(inc.MAZEWIDTH+2):
        print 'X',
    print
    for y in range(inc.MAZEHEIGHT):
        print 'X',
        for x in range(inc.MAZEWIDTH):
            if y == mazegoal.y and x == mazegoal.x:
                print 'R',
            elif y == mazestart.y and x == mazestart.x:
                print 'G',
            elif maze[y][x].obstacle == 1:
                print 'X',
            else:
                print ' ',
        print 'X'

    for x in range(inc.MAZEWIDTH+2):
        print 'X',
    print '\n--------------------------------------------------------------------\n'


def printknownmaze():
    global maze
    global mazestart
    global mazegoal
    display = None

    # rospy.logwarn('printknownmaze() called')

    if (display == None):
        display = []
        for y in range(inc.MAZEHEIGHT):
            column = []
            for x in range(inc.MAZEWIDTH):
                column.append('')
            display.append(column)

    for y in range(inc.MAZEHEIGHT):
        for x in range(inc.MAZEWIDTH):
            display[y][x] = 'X'
            for d in range(inc.DIRECTIONS):
                if maze[y][x].move[d]:
                    display[y][x] = ' '

    tmpcell = mazegoal
    while tmpcell != mazestart:
        display[tmpcell.y][tmpcell.x] = '.'
        tmpcell = tmpcell.searchtree
    display[mazestart.y][mazestart.x] = 'G'
    display[mazegoal.y][mazegoal.x] = 'R'

    for x in range(inc.MAZEWIDTH+2):
	    print 'X',
    print
    for y in range(inc.MAZEHEIGHT):
        print 'X',
        for x in range(inc.MAZEWIDTH):
            print display[y][x],
        print 'X'
    for x in range(inc.MAZEWIDTH+2):
	    print 'X',
    print '\n\n'


def initialize():
    global mazestart
    global mazegoal
    global keymodifier

    initMaze.mazeiteration = initMaze.mazeiteration + 1
    keymodifier = 0
    mazestart.g = inc.LARGE
    mazestart.rhs = 0
    if inc.TIEBREAKING:
        initHeap.emptyheap(3)
        mazestart.key[0] = heuristic(mazestart)
        mazestart.key[1] = heuristic(mazestart) + 1
        mazestart.key[2] = heuristic(mazestart)
    else:
        initHeap.emptyheap(2)
        mazestart.key[0] = heuristic(mazestart)
        mazestart.key[1] = 0

    mazestart.searchtree = None
    mazestart.generated = initMaze.mazeiteration
    initHeap.insertheap(mazestart)
    mazegoal.g = inc.LARGE
    mazegoal.rhs = inc.LARGE
    mazegoal.searchtree = None
    mazegoal.generated = initMaze.mazeiteration

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
    if thiscell.generated != initMaze.mazeiteration:
        thiscell.g = inc.LARGE
        thiscell.rhs = inc.LARGE
        thiscell.searchtree = None
        thiscell.generated = initMaze.mazeiteration


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
        initHeap.insertheap(thiscell)

    elif thiscell.g > thiscell.rhs:
        if inc.TIEBREAKING:
            thiscell.key[0] = thiscell.rhs + heuristic(thiscell) + keymodifier
            thiscell.key[1] = thiscell.rhs + heuristic(thiscell) + keymodifier + 1
            thiscell.key[2] = heuristic(thiscell) + keymodifier
        else:
            thiscell.key[0] = thiscell.rhs + heuristic(thiscell) + keymodifier
            thiscell.key[1] = thiscell.rhs
        initHeap.insertheap(thiscell)

    else:
	    initHeap.deleteheap(thiscell)


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

    '''
    if RANDOMIZESUCCS:
        int dcase, dtemp
    #endif
    '''
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
        if thiscell.move[d] and thiscell.move[d].generated == initMaze.mazeiteration and thiscell.rhs > thiscell.move[d].g + 1:
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
    goaltmpcell = initMaze.Cell()
    oldtmpcell  = initMaze.Cell()
    '''
    #ifdef RANDOMIZESUCCS
        int dcase, dtemp
    #endif
    '''
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

    while initHeap.topheap() and (mazegoal.rhs > mazegoal.g or initHeap.keyless(initHeap.topheap(), goaltmpcell)):
        tmpcell1 = initHeap.topheap()
        oldtmpcell.key[0] = tmpcell1.key[0]
        oldtmpcell.key[1] = tmpcell1.key[1]

        if inc.TIEBREAKING:
            oldtmpcell.key[2] = tmpcell1.key[2]

        updatekey(tmpcell1)
        if initHeap.keyless(oldtmpcell, tmpcell1):
            updatecell(tmpcell1)
        elif tmpcell1.g > tmpcell1.rhs:
            tmpcell1.g = tmpcell1.rhs
            initHeap.deleteheap(tmpcell1)

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
        else:
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
    '''
    #ifdef RANDOMIZESUCCS
        int dcase, dtemp
    #endif
    '''

    #ifdef RANDOMIZESUCCS
    dcase = random.randint(0, permutations-1)
    for dtemp in range(inc.DIRECTIONS):
        d1 = permutation[dtemp][dcase]
        '''
        #else
        for d1 in range(inc.DIRECTIONS):
        '''
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

#ifdef RANDOMIZESUCCS
createpermutations()
#endif

keymodifier = 0

for k in range(inc.RUNS):
    print '====================================================================='
    print 'maze', k
    '''
    #ifdef inc.RANDOMMAZE:
        initMaze.newrandommaze()
    #endif:
    '''
    initMaze.newdfsmaze(inc.WALLSTOREMOVE)
    maze = initMaze.maze
    mazestart   = initMaze.mazestart
    mazegoal    = initMaze.mazegoal

    if inc.DISPLAY:
        printactualmaze()

    initialize()
    lastcell = mazegoal
    while mazestart != mazegoal:
        if computeshortestpath():
            print 'breaaaaak'
            break
        if inc.DISPLAY:
            printknownmaze()

        mazegoal.trace = None
        while True:
            mazegoal.searchtree.trace = mazegoal
            mazegoal = mazegoal.searchtree
            if mazestart == mazegoal or mazegoal.searchtree.obstacle:
                break

        if mazestart != mazegoal:
            keymodifier = keymodifier + heuristic(lastcell)
            lastcell = mazegoal

            tmpcell = mazegoal
            while tmpcell:
                updatemaze(tmpcell)
                tmpcell = tmpcell.trace
    # print initHeap.heap
