''' D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) '''
import rospy
import random

LARGE = 1000000

DISPLAY = True                  # display what happens (in ASCII)                                                     */
RANDOMIZESUCCS = True           # randomize the order in which successors of a node are generated                     */
# WALLSTOREMOVE = 4             # number of walls to remove if RANDOMMAZE is NOT defined - infinite loop if too large */
# DEBUG = True                  # whether debugging is on - debugging takes time but performs various checks          */
MAZELENGTH	= 51                # the length of the maze                                                               */
MAZEWIDTH 	= 51                # the width of the maze
MAZEHEIGHT	= 31
MAZEDENSITY = 0.10              # percentage of blocked cells if RANDOMMAZE is defined                                */
STARTX = random.randint(0,5)    # x coordinate of the start cell                                                      */
STARTY = random.randint(0,5)    # y coordinate of the start cell
STARTZ = random.randint(0,5)
GOALX = random.randint(int(0.8*MAZELENGTH),MAZELENGTH-1)   # x coordinate of the goal  cell                                                      */
GOALY = random.randint(int(0.8*MAZEWIDTH), MAZEWIDTH-1)   # y coordinate of the goal  cell
GOALZ = random.randint(int(0.8*MAZEHEIGHT),MAZEHEIGHT-1)
INFORMEDSEARCH = True           # use Manhattandistance rather than zero heuristics                                   */
RUNS = 3                        # number of different runs                                                            */
TIEBREAKING = True              # tie breaking towards larger g-values (otherwise: smaller g-values)                  */

DIRECTIONS = 6
dx = [1, 0, 0, -1,  0,  0]
dy = [0, 1, 0,  0, -1,  0]
dz = [0, 0, 1,  0,  0, -1]
reverse = [3, 4, 5, 0, 1, 2]
