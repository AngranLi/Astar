''' D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) '''
import rospy
import random

LARGE = 1000000

DISPLAY = True                  # display what happens (in ASCII)                                                     */
RANDOMIZESUCCS = True           # randomize the order in which successors of a node are generated                     */
# WALLSTOREMOVE = 4             # number of walls to remove if RANDOMMAZE is NOT defined - infinite loop if too large */
# DEBUG = True                  # whether debugging is on - debugging takes time but performs various checks          */
MAZEWIDTH	= 21                # the width of the maze                                                               */
MAZEHEIGHT 	= 21                # the height of the mze                                                               */
MAZEDENSITY = 0.10              # percentage of blocked cells if RANDOMMAZE is defined                                */
STARTX = random.randint(0,5)    # x coordinate of the start cell                                                      */
STARTY = random.randint(0,5)    # y coordinate of the start cell                                                      */
GOALX = random.randint(5,20)   # x coordinate of the goal  cell                                                      */
GOALY = random.randint(15,20)   # y coordinate of the goal  cell                                                      */
INFORMEDSEARCH = True           # use Manhattandistance rather than zero heuristics                                   */
RUNS = 3                        # number of different runs                                                            */
TIEBREAKING = True              # tie breaking towards larger g-values (otherwise: smaller g-values)                  */

DIRECTIONS = 8
dx = [1, 1, 0, -1, -1, -1,  0,  1]
dy = [0, 1, 1,  1,  0, -1, -1, -1]
reverse = [4, 5, 6, 7, 0, 1, 2, 3]
