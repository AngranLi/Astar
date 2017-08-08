''' D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) '''

# include <stdio.h>
# include <stdlib.h>
# include <assert.h>
import rospy

LARGE = 1000000

DISPLAY = True                  # display what happens (in ASCII)                                                     */
RANDOMIZESUCCS = True           # randomize the order in which successors of a node are generated                     */
# RANDOMMAZE                    # whether the gridworld has random obstacles or is a maze created with dfs            */
WALLSTOREMOVE = 4               # number of walls to remove if RANDOMMAZE is NOT defined - infinite loop if too large */
# DEBUG                         # whether debugging is on - debugging takes time but performs various checks          */
MAZEWIDTH = 21                  # the width of the maze                                                               */
MAZEHEIGHT = 21                 # the height of the mze                                                               */
MAZEDENSITY = 0.25              # percentage of blocked cells if RANDOMMAZE is defined                                */
# STARTCANBEBLOCKED             # whether the goal cell of the robot can be blocked                                   */
RANDOMSTARTGOAL = True          # whether the start and goal state are drawn randomly                                 */
STARTX = 0                      # x coordinate of the start cell                                                      */
STARTY = 0                      # y coordinate of the start cell                                                      */
GOALX = 20                      # x coordinate of the goal  cell                                                      */
GOALY = 20                      # y coordinate of the goal  cell                                                      */
INFORMEDSEARCH = True           # use Manhattandistance rather than zero heuristics                                   */
RUNS = 5                        # number of different runs                                                            */
TIEBREAKING = True              # tie breaking towards larger g-values (otherwise: smaller g-values)                  */

DIRECTIONS = 4
'''3 static lists'''
dx = [1, 0, -1,  0]
dy = [0, 1,  0, -1]
reverse = [2, 3, 0, 1]


# if INFORMEDSEARCH:
# def heuristic(cell):
#     global mazegoal
#     return abs(cell.y-mazegoal.y) + abs(cell.x-mazegoal.x)
