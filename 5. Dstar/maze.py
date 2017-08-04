''' D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) '''

import include

class cell():
    def __init__(self):
        ''' 4 pointers '''
        self.move = []*DIRECTIONS
        self.succ = []*DIRECTIONS
        self.searchtree = []
        self.trace = []

        self.obstacle = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.dfsx = 0
        self.dfsy = 0
        self.dfsz = 0
        self.g = float('inf')
        self.rhs = float('inf')
        self.key = []*3
        self.generated = 0
        self.heapindex = 0

# Note: mazegoal is the start cell of the robot.
# Note: mazestart is the goal cell of the robot.

# cell **maze
# cell *mazestart, *mazegoal

mazeiteration = 0

'''maze.c'''

# cell **maze = NULL
# cell *mazestart, *mazegoal
mazeiteration = 0

goaly = GOALY
goalx = GOALX
starty = STARTY
startx = STARTX

def preprocessmaze():
    int x, y, d
    int newx, newy

    maze = []
    for y in range(MAZEHEIGHT):
        column = []
	    for x in range(MAZEWIDTH):
            cell = cell()
            cell.x = x
            cell.y = y
        for d in range(DIRECTIONS):
            newy = y + dy[d]
            newx = x + dx[d]
            if (newy >= 0 and newy < MAZEHEIGHT and newx >= 0 and newx < MAZEWIDTH):
                cell.succ[d] = &maze[newy][newx]
            else:
                cell.succ[d] = None
            column.append(cell)
        maze.append(column)
    '''
    if DEBUG:
        assert(STARTY % 2 == 0)
        assert(STARTX % 2 == 0)
        assert(GOALY % 2 == 0)
        assert(GOALX % 2 == 0)
    '''
    mazestart = &maze[STARTY][STARTX]
    mazegoal = &maze[GOALY][GOALX]
    mazeiteration = 0



def postprocessmaze():
    int x, y
    int d1, d2
    cell *tmpcell

    for y in range (MAZEHEIGHT):
        for x in range(MAZEWIDTH):
            maze[y][x].generated = 0
            maze[y][x].heapindex = 0
            for d1 in range(DIRECTIONS):
                maze[y][x].move[d1] = maze[y][x].succ[d1]

    for d1 in range(DIRECTIONS):
        if (mazegoal->move[d1] && mazegoal->move[d1]->obstacle):
            tmpcell = mazegoal->move[d1]
            for d2 in range(DIRECTIONS):
                if (tmpcell->move[d2]):
                    tmpcell->move[d2] = NULL
                    tmpcell->succ[d2]->move[reverse[d2]] = None


def newrandommaze():
    int d1, d2
    int x, y
    int newx, newy
    cell *tmpcell

    preprocessmaze()
    for y in range (MAZEHEIGHT):
        for x in range(MAZEWIDTH):
	    maze[y][x].obstacle = (random() % 10000 < 10000 * MAZEDENSITY)
    mazegoal->obstacle = 0

    if STARTCANBEBLOCKED:
        mazestart->obstacle = 0
    postprocessmaze()


def newdfsmaze(wallstoremove):
    int d, dtmp
    int x, y
    int newx, newy
    int randomnumber
    cell *tmpcell
    int permute[8] = {0, 1, 2, 3, 4, 5, 6, 7}
    int permutetmp

    preprocessmaze()
    if DEBUG:
        assert(MAZEWIDTH % 2 == 1)
        assert(MAZEHEIGHT % 2 == 1)

    for y in range (MAZEHEIGHT):
        for x in range(MAZEWIDTH):
    	    maze[y][x].obstacle = 1
    	    maze[y][x].dfsx = 0
    	    maze[y][x].dfsy = 0
    x = 0
    y = 0
    maze[y][x].dfsx = -1
    maze[y][x].dfsy = -1
    while True:
        if (maze[y][x].obstacle):
            maze[y][x].obstacle = 0
        for (d = 0; d < DIRECTIONS-1; ++d):
            randomnumber = random() % (DIRECTIONS-d)
            permutetmp = permute[randomnumber]
            permute[randomnumber] = permute[DIRECTIONS-1-d]
            permute[DIRECTIONS-1-d] = permutetmp

        for (dtmp = 0; dtmp < DIRECTIONS; ++dtmp):
            d = permute[dtmp]
            newx = x + 2*dx[d]
            newy = y + 2*dy[d]
            if (maze[y][x].succ[d] != NULL && maze[newy][newx].obstacle):
                if (maze[y + dy[d]][x + dx[d]].obstacle)
                    maze[y + dy[d]][x + dx[d]].obstacle = 0
                maze[newy][newx].dfsx = x
                maze[newy][newx].dfsy = y
                x = newx
                y = newy
                break

        if (dtmp == DIRECTIONS):
            if (maze[y][x].dfsx == -1):
                break
            newx = maze[y][x].dfsx
            newy = maze[y][x].dfsy
            x = newx
            y = newy

    while wallstoremove:
    	newx = random() % MAZEWIDTH
    	newy = random() % MAZEHEIGHT
    	if (maze[newy][newx].obstacle):
    	    maze[newy][newx].obstacle = 0
    	    --wallstoremove
    mazegoal->obstacle = 0

    if STARTCANBEBLOCKED:
        mazestart->obstacle = 0

    postprocessmaze()
