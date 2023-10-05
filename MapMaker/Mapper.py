from robot import *
from show_map import ShowMap
import numpy as np
import math
from timeit import default_timer as timer

"""
*   Controlling program for Kompai Robot in MRDS environment
    for the "Artificial Intelligence - Methods and Applications" course
    at the Department of Computing Science, Umeå University.
    The Robot explores a given area and mapps it.

*   Authors: Matilda Sandberg (tfy17msg@cs.umu.se)
             Jiangeng Sun (mrc20jsn@cs.umu.se)

*   Version information:
             v1.0  2021-01-08: First version
"""

class Grid:
    # Initialize the grid
    def __init__(self, nRows, nCols):
        self.nRows = nRows
        self.nCols = nCols
        # This will create a matrix from (0,0) to (nRows-1,nCols-1)
        # If the value of the square is 7 then it is unobserved
        self.grid = np.ones(shape=(nRows, nCols)) * 7

    # Get number of unobserved squares in the grid
    def get_nUnobserved_squares(self):
        nUnobserved = 0
        # Loop through the grid (from position (0,0) to (nRows-1,nCols-1)
        for x in range(self.nRows):
            for y in range(self.nCols):
                # If position is surrounded by obstacles it is not considered unobserved
                dir = np.ones(4, dtype = int)*15
                if y-1 >= 0:
                    dir[0] = self.grid[x][y-1]
                if y+1 < self.nCols:
                    dir[1] = self.grid[x][y+1]
                if x+1 < self.nRows:
                    dir[2] = self.grid[x+1][y]
                if x-1 >= 0:
                    dir[3] = self.grid[x-1][y]

                if self.grid[x][y] == 7 and not all(i >= 7 for i in dir):
                    nUnobserved += 1
        return nUnobserved

def pos_to_grid(x, y, xmin, ymax, cellsize):
    """
    Converts an (x,y) positon to a (row,col) coordinate in the grid
    :param x: x-position
    :param y: y-position
    :param xmin: The minimum x-position in the grid
    :param ymax: The maximum y-position in the grid (the reason it is ymax is because rows increase in negative y-direction)
    :param cellsize: the resolution of the grid (meters per grid cell)
    :return: A tuple with (row,col)
    """
    # math.floor to get integers
    col = math.floor((x - xmin) / cellsize)
    row = math.floor((ymax - y) / cellsize)
    return (row, col)

def grid_to_pos(row, col, xmin, ymax, cellsize):
    """
    Converts an (x,y) positon to a (row,col) coordinate in the grid
    :param row: The row
    :param col: The column
    :param xmin: The minimum x-position in the grid
    :param ymax: The maximum y-position in the grid (the reason it is ymax is because rows increase in negative y-direction)
    :param cellsize: the resolution of the grid (meters per grid cell)
    :return: A tuple with (x,y) position in world coordinate system
    """
    x = col * cellsize + xmin
    y = ymax - row * cellsize
    return (x, y)

def bresenham(x0, y0, x1, y1):
    """
    Finds coordinates of points on line between two points
    :param x0: x-coordinate of one edge of the line
    :param y0: y-coordinate corresponding to x0
    :param x1: x-coordinate of other edge of the line
    :param y1: y-coordinate corresponding to x1
    :return: A list of x coordinates and a list of y coordinates corresponding to points on the line between the points (x0,y0) and (x1,y1)
    """
    xlist = []
    ylist = []

    # If change in y us bigger than change in x rotate line
    steep = False
    if abs(y1-y0) > abs(x1-x0):
        x0, y0 = y0, x0
        x1, y1 = y1, x1
        steep = True

    # Make sure x0 < x1
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0

    # Initialize values
    error = math.floor((x1 - x0) / 2)
    y = y0

    # If change in x is greater than change in y
    if (x1 - x0) != 0:
        slope = (y1 - y0) / (x1 - x0)
        for x in range(x0,x1):
            if x != x0 and x != x1:
                if steep:
                    xlist.append(math.floor(y))
                    ylist.append(math.floor(x))
                else:
                    xlist.append(math.floor(x))
                    ylist.append(math.floor(y))
            error -= abs(y1-y0)
            if error < 0:
                y += np.sign(slope)
                error += x1-x0
    return xlist, ylist

def neighbours(my_grid, row, col):
    """
    Finds neighbours of position (row, col) in my_grid in the directions that neighbours exist
    :param my_grid: The grid
    :param row: The row
    :param col: The coordinate
    :return: A list of neighbouring gridpoints
    """
    neighbour_list = []
    if col+1 < my_grid.nCols:
        neighbour_list.append([row,col+1])
    if row+1 < my_grid.nRows:
        neighbour_list.append([row+1,col])
    if row+1 < my_grid.nRows and col+1 < my_grid.nCols:
        neighbour_list.append([row+1,col+1])
    if col-1 >= 0:
        neighbour_list.append([row,col-1])
    if row-1 >= 0:
        neighbour_list.append([row-1,col])
    if row-1 >= 0 and col-1 >= 0:
        neighbour_list.append([row-1,col-1])
    if row-1 >= 0 and col+1 < my_grid.nCols:
        neighbour_list.append([row-1,col+1])
    if row+1 < my_grid.nRows and col-1 >= 0:
        neighbour_list.append([row+1,col-1])
    return neighbour_list

def cartographer(my_grid):
    """
    The cartographer updates the grid depending on laser input
    :param my_grid: The current grid
    :return: The updaed grid
    """
    # Get the angle of each laser beam (0-270) in radians
    laser_angles = robot.getLaserAngles()
    # Get the distances of all laser beams, laser_scan['Echoes'][i], i = 135 straight ahead
    laser_scan = robot.getLaser()
    # Get the current position of the Robot, has ['X'] and ['Y'] value
    curr_pos = robot.getPosition()
    # Get te current heading of the Robot in radians from the WCS x-axis counter clockwise
    curr_heading = robot.getHeading()
    # Transfer the robot's position to grid coordinates
    robot_coord = pos_to_grid(curr_pos['X'], curr_pos['Y'], xmin, ymax, cellsize)
    # The place where the robot is is empty
    my_grid.grid[robot_coord[0]][robot_coord[1]] = 0
    # For all beams
    for i in range(271):
        # Object pos for all 270 beams, obj_pos[0] is X-position and obj_pos[1] is Y-position
        obj_pos = [curr_pos['X'] + laser_scan['Echoes'][i] * math.cos((curr_heading + laser_angles[i]) % (2*pi)), curr_pos['Y'] + laser_scan['Echoes'][i] * math.sin((curr_heading + laser_angles[i]) % (2*pi))]
        # Transfer the object's position to grid coordinates
        object_coord = pos_to_grid(obj_pos[0], obj_pos[1], xmin, ymax, cellsize)
        # Obstacles need to be closer to be considered seen
        if laser_scan['Echoes'][i] < 39:
            # Make sure that what's seen is within the grid
            if object_coord[0] < nRows and object_coord[0] >= 0 and object_coord[1] < nCols and object_coord[1] >= 0:
                # Update grid
                [xlisttest, ylisttest] = bresenham(robot_coord[0], robot_coord[1], object_coord[0], object_coord[1])
                keepgoing = True
                for j in range(len(xlisttest)):
                    # If obstacle between then something is wrong
                    if my_grid.grid[xlisttest[j]][ylisttest[j]] > 11:
                        keepgoing = False
                        break
                if keepgoing:
                    if my_grid.grid[object_coord[0]][object_coord[1]] <= 12:
                        my_grid.grid[object_coord[0]][object_coord[1]] += 3
                    else:
                        my_grid.grid[object_coord[0]][object_coord[1]] = 15
                # Mask
                sum = 0
                neighbour_list = neighbours(my_grid, object_coord[0], object_coord[1])
                for neighbour in neighbour_list:
                    sum += my_grid.grid[neighbour[0]][neighbour[1]]*0.4
                my_grid.grid[object_coord[0]][object_coord[1]] = min(15,sum)

        # Add empty parts
        if laser_scan['Echoes'][i] < 44:
            # Use Bresenham's algorithm to update squares between object and robot
            xlist1 = []
            ylist1 = []
            x0 = robot_coord[0]
            x1 = object_coord[0]
            y0 = robot_coord[1]
            y1 = object_coord[1]
            [xlist1, ylist1] = bresenham(x0, y0, x1, y1)
            # Update grid
            for i in range(len(xlist1)):
                if 0 <= xlist1[i] < my_grid.nRows and 0 <= ylist1[i] < my_grid.nCols and my_grid.grid[xlist1[i]][ylist1[i]] >= 1:
                    if my_grid.grid[xlist1[i]][ylist1[i]] != 15: # If it once has become 15 it is an obstacle
                        my_grid.grid[xlist1[i]][ylist1[i]] -= 1
                    else:
                        break

    return my_grid

def frontier(position,thisgrid, unreachable):
    """
    Checks if the gridpoint is a fronter point (empty with unseen neighbour)
    :param position: The row and column of the gridposition we want to check
    :param thisgrid: The grid
    :param unreachable: A list of positions that from the robots current position couldn't be reached (the navigator didn't find path)
    :return: True if frontier point, if not False
    """
    this_grid = thisgrid.grid
    if position[0] <= 0 or position[0] >= thisgrid.nRows-1:
        return False
    elif position[1] <= 0 or position[1] >= thisgrid.nCols-1:
        return False
    else:
        gridlist = np.ones(8, dtype = int)
        if position[1] - 1 >= 0:
            west = (position[0],position[1] - 1)
            gridlist[0] = this_grid[west]
        if position[1] + 1 < thisgrid.nCols:
            east = (position[0],position[1] + 1)
            gridlist[1] = this_grid[east]
        if position[0] + 1 < thisgrid.nRows:
            south = (position[0] + 1,position[1])
            gridlist[2] = this_grid[south]
        if position[0] - 1 >= 0:
            north = (position[0] - 1,position[1])
            gridlist[3] = this_grid[north]

        if position[1] - 1 >= 0 and position[0] + 1 < thisgrid.nRows:
            southwest = (position[0]+1,position[1] - 1)
            gridlist[4] = this_grid[southwest]
        if position[1] + 1 < thisgrid.nCols and position[0] + 1 < thisgrid.nRows:
            eastsouth = (position[0]+1,position[1] + 1)
            gridlist[5] = this_grid[eastsouth]
        if position[1] - 1 >= 0 and position[0] - 1 >= 0:
            westnorth = (position[0]-1,position[1] - 1)
            gridlist[6] = this_grid[westnorth]
        if position[1] + 1 < thisgrid.nCols and position[0] - 1 >= 0:
            eastnorth = (position[0]-1,position[1] + 1)
            gridlist[7] = this_grid[eastnorth]


        nowposition=(position[0],position[1])
        # To avoid going to frontierpoints which can't be reached
        pos = [position[0], position[1]]
        if pos in unreachable:
            return False
        for check in range(4):
            if gridlist[check] > 7:
                return False
        # If empty position with unseen neighbour
        if 7 in gridlist or 8 in gridlist or 6 in gridlist:
            if this_grid[nowposition] <= 4:
                return True
    return False

def findgoal(RobotPosition,ThisGrid, startpos, unreachable):
    """
    Finds the goal point to which the robot should move by choosing closest frontier points
    :param RobotPosition: The position of the robot
    :param ThisGrid: The grid
    :param startpos: If no goal point can be found we choose the startpos as goal point (if this happens many times we will be done with the map)
    :param unreachable: A list of positions that from the robots current position couldn't be reached (the navigator didn't find path)
    :return: The goal point as [row, col]
    """
    pose = [RobotPosition[0],RobotPosition[1]]
    queue_m = []
    queue_m.append(pose)
    Map_open_list=[]
    Map_close_list=[]
    Frontier_open_list=[]
    Frontier_close_list=[]
    NewFrontier = []
    x_list = []
    y_list = []
    Map_open_list.append(pose)
    while len(queue_m) > 0:
        p = queue_m.pop(0)
        if p in Map_close_list:
            continue
        if frontier(p,ThisGrid, unreachable):
            queue_f = []
            queue_f.append(p)
            Frontier_open_list.append(p)
            while len(queue_f) > 0:
                q = queue_f.pop(0)
                if q in Map_close_list or q in Frontier_close_list:
                    continue
                if frontier(q,ThisGrid, unreachable):
                    NewFrontier.append(q)
                    w = [[q[0]+1,q[1]],[q[0]-1,q[1]],[q[0],q[1]+1],[q[0],q[1]-1]]
                    for i in range(len(w)):
                        if w[i] not in Frontier_open_list and w[i] not in Frontier_close_list and w[i] not in Map_close_list:
                            queue_f.append(w[i])
                            Frontier_open_list.append(w[i])
                Frontier_close_list.append(q)
                Frontier_open_list.remove(q)
            for l in range(len(NewFrontier)):
                NewFrontier_L = NewFrontier[l]
                x_list.append(NewFrontier_L[0])
                y_list.append(NewFrontier_L[1])
            for j in range(len(NewFrontier)):
                Map_close_list.append(NewFrontier[j])
            if len(NewFrontier)<=3:
                x_list=[]
                y_list=[]
                NewFrontier=[]
                continue
            break

        v = [[p[0]+1, p[1]], [p[0]-1, p[1]], [p[0], p[1]+1], [p[0], p[1]-1]]
        if p[0] == 0 or p[1] == ThisGrid.nRows-1 or p[1]==0 or p[1]== ThisGrid.nCols-1:
            continue
        for k in range (len(v)):
            v_k = v[k]
            v_1 = [v_k[0]+1, v_k[1]]
            v_2 = [v_k[0]-1, v_k[1]]
            v_3 = [v_k[0], v_k[1]+1]
            v_4 = [v_k[0], v_k[1]-1]
            if v_k[0]==0 or v_k[0]== ThisGrid.nRows-1 or v_k[1] == 0 or v_k == ThisGrid.nCols-1:
                continue
            if v_k not in Map_open_list and v_k not in Map_close_list:
                if v_1 in Map_open_list or v_2 in Map_open_list or v_3 in Map_open_list or v_4 in Map_open_list:
                    queue_m.append(v_k)
                    Map_open_list.append(v_k)
                continue
        Map_close_list.append(p)
        Map_open_list.remove(p)

    Distance=[]
    for o in range(len(x_list)):
        DistanceL_x =  np.square(x_list[o]-RobotPosition[0])
        DistanceL_y =  np.square(y_list[o]-RobotPosition[1])
        DistanceL = math.sqrt(DistanceL_x + DistanceL_y)
        Distance.append(DistanceL)
    if len(Distance) != 0:
        min_index = Distance.index(min(Distance))
        Goal_point=[x_list[min_index],y_list[min_index]]
    else:
        Goal_point=startpos
    return Goal_point

def navigator(grid,start_point,goal_point):
    """
    Finds the path to the goal point from the robots position
    :param grid: The grid
    :param start_point: The point from which the path starts (the robot's position)
    :param goal_point: The at which the path sould end
    :return: The path as two lists, one of row coordinates (here called save_x) and one of column coordinates (here called save_y) or two empty lists if path couldn't be found
    """
    new_grid = np.ones(shape=(nRows, nCols)) * 7
    start_point = [start_point[0],start_point[1]]
    if start_point == goal_point:
        return [],[]
    # remake the grid
    index_far = 2
    for i in range(grid.nRows):
        for j in range(grid.nCols):
            if grid.grid[i,j] > 7:
                new_grid[i,j] = -100
                # Don't want to make obstacles bigger around goal point
                if abs(goal_point[0]-i) < index_far and abs(goal_point[1]-j) < index_far:
                    continue
                # Don't want to make obstacles bigger around robots current position
                if abs(start_point[0]-i) < index_far and abs(start_point[1]-j) < index_far:
                    continue


                # To make robot move further away from obstacles make the obstacles bigger
                for nn in range(1,index_far):
                    if j+nn < grid.nCols:
                        new_grid[i,j+nn] = -100
                    else:
                        continue
                for mm in range(1,index_far):
                    if j-mm >= 0:
                        new_grid[i,j-mm] = -100
                    else:
                        continue
                for kk in range(1,index_far):
                    if i+kk < grid.nRows:
                        new_grid[i+kk,j] = -100
                    else:
                        continue
                for ll in range(1,index_far):
                    if i-ll >= 0:
                        new_grid[i-ll,j] = -100
                    else:
                        continue


            elif grid.grid[i,j] == 7:
                new_grid[i,j] = -50
            elif grid.grid[i,j] < 7:
                new_grid[i,j] = 0

    # set start_point
    new_grid[start_point[0],start_point[1]] = 1
    # set GoalPoint
    new_grid[goal_point[0],goal_point[1]] = 65000
    # If surrounded by obstacle return empty vectors
    if new_grid[goal_point[0]+1][goal_point[1]] <0 and new_grid[goal_point[0]][goal_point[1]+1] <0 and new_grid[goal_point[0]-1][goal_point[1]] <0 and new_grid[goal_point[0]][goal_point[1]-1] <0:
        return [], []
    # add the rows
    my_grid_add = np.ones(shape=grid.nCols) * -50
    new_grid = np.insert(new_grid, 0, values=my_grid_add, axis=0)
    new_grid = np.insert(new_grid, grid.nRows+1, values=my_grid_add, axis=0)
    # add the cols
    my_grid_add = np.ones(shape=grid.nRows+2) * -50
    new_grid = np.insert(new_grid, 0, values=my_grid_add, axis=1)
    new_grid = np.insert(new_grid, grid.nCols+1, values=my_grid_add, axis=1)

    # start to search
    countingloops = 0
    switch = True
    while switch == True:
        countingloops += 1
        for i in range(0, grid.nRows+1):
            for j in range(0, grid.nCols+1):
                east = new_grid[i, j + 1]
                west = new_grid[i, j - 1]
                north = new_grid[i - 1, j]
                south = new_grid[i + 1, j]
                # if it is the point where robot is
                if new_grid[i, j] == 1:
                    continue
                # if it is a occupied point or unknown region
                if new_grid[i, j] < 0:
                    continue
                # if it is the goal point
                if new_grid[i, j] == 65000:
                    if north > 0 or south > 0 or west > 0 or east > 0:
                        final_grid = new_grid[1:grid.nRows+1,1:grid.nCols+1]
                        # From the goal point, check the path
                        save_x = []
                        save_y = []
                        a = goal_point[0]+1
                        b = goal_point[1]+1
                        # Here new_grid is used to make all points have neighbours
                        # Check the lower value from the goal point
                        while new_grid[a][b] != 1:
                            up = new_grid[a-1, b]
                            down = new_grid[a+1, b]
                            left = new_grid[a, b-1]
                            right = new_grid[a, b+1]
                            if up <= 0:
                                up = 1000000
                            if down <= 0:
                                down = 1000000
                            if left <= 0:
                                left = 1000000
                            if right <= 0:
                                right = 1000000
                            KeyValue = min(up,down,left,right)
                            if up == KeyValue:
                                a = a - 1
                                b = b
                                save_x.append(a)
                                save_y.append(b)
                                continue
                            if down == KeyValue:
                                a = a + 1
                                b = b
                                save_x.append(a)
                                save_y.append(b)
                                continue
                            if right == KeyValue:
                                a = a
                                b = b + 1
                                save_x.append(a)
                                save_y.append(b)
                                continue
                            if left == KeyValue:
                                a = a
                                b = b - 1
                                save_x.append(a)
                                save_y.append(b)
                                continue
                        save_x = save_x[::-1]
                        save_y = save_y[::-1]
                        for m in range(len(save_x)):
                            save_x[m] = save_x[m]-1
                        for n in range(len(save_y)):
                            save_y[n] = save_y[n]-1
                        if len(save_x) != 0 and len(save_y) != 0:
                            return save_x,save_y
                    # else, if it is just a goal point without valued neighbours,continue
                    else:
                        continue
                # else, if it is the neighbour point of the robot position
                elif north == 1 or south == 1 or west == 1 or east == 1:
                    new_grid[i, j] = 2
                    continue
                # else, if its neighbours has a certain value
                elif 65000 > north > 1 or 65000 > south > 1 or 65000 > west > 1 or 65000 > east > 1:
                    if north <= 0:
                        north = 1000000
                    if south <= 0:
                        south = 1000000
                    if west <= 0:
                        west = 1000000
                    if east <= 0:
                        east = 1000000
                    new_grid[i, j] = min(north,south,west,east) + 1
        # if after searching all grids, still not find the path
        # if still some empty grid not been searched or maximum number of searched not reached, continue
        if 0 in new_grid and countingloops < 120:
            switch = True
            continue
        else:
            switch = False
            return [], []

def outerloop(my_grid, robot, startpos, xmin, xmax, ymin, ymax, cellsize, time_limit, v, startposcounter,goalpoint, unreachable):
    """
    The outer loop from wich the robot is controlled while mapping. Finds goal, path and moves there while mapping.
    :params: See definition of parameters where the functions are called
    :param startposcounter: If the goal point is the start point multiple times this means it cannot find any more goal points and we sould end the mapping
    :param unreachable: Vector of previously reached goal points and points to which the navigator doesn't find a path to avoid choosing the same
    :return: False if startposcounter is high enough, else true. startposcounter is also returned
    """
    # Stop robot and update map
    robot.setMotion(0, 0)
    curr_pos = robot.getPosition()
    robot_coord = pos_to_grid(curr_pos['X'], curr_pos['Y'], xmin, ymax, cellsize)
    map.updateMap(my_grid.grid, maxVal, robot_coord[0], robot_coord[1])
    found = False
    whilecount = 0
    while found == False:
        GoalPoint = findgoal(robot_coord,my_grid, startpos, unreachable)
        # If te goal point is the same as startpos that is because findgoal can't find a goal point
        if startpos == GoalPoint:
            startposcounter += 1
            if startposcounter == 2:
                return False, startposcounter, GoalPoint, unreachable
        # If the goal point is the same as last goal point, get new one
        if GoalPoint == goalpoint:
            unreachable.append(GoalPoint)
            GoalPoint = findgoal(robot_coord,my_grid, startpos, unreachable)
        r,c = navigator(my_grid,robot_coord,GoalPoint)
        # If no path was found, get new goal point
        if len(r) == 0:
            unreachable.append(GoalPoint)
            my_grid.grid[GoalPoint[0]][GoalPoint[1]] = 10 # We don' want the goal point to keep being empty if not reachable
            my_grid = cartographer(my_grid)
        else:
            found = True
        whilecount += 1
        # If it hasn't found a place it can go to in 3 tries go back to startposition and try again from there
        if whilecount == 3:
            GoalPoint = startpos
            found = True
    # If you don't want to be able to choose same goal point many times uncomment line below
    # unreachable.append(GoalPoint)
    pathxx = []
    pathyy = []
    for i in range(1,len(r)):  # From 1 to ignore the robots current position
        path = grid_to_pos(r[i], c[i], xmin, ymax, cellsize)
        pathxx.append(path[0])
        pathyy.append(path[1])

    goalposition = grid_to_pos(GoalPoint[0], GoalPoint[1], xmin, ymax, cellsize)
    pathxx.append(goalposition[0])
    pathyy.append(goalposition[1])
    # For the first point on the path turn the robot around
    firstpointonpath(my_grid, robot, pathxx[0],pathyy[0])
    # For all points on the path
    for lookaheadpoint in range(len(pathxx)):
        if my_grid.get_nUnobserved_squares() == 0:
            break
        # If purepuirsuit returns true an obstacle is in the way
        bool = purepuirsuit(my_grid, robot, pathxx[lookaheadpoint], pathyy[lookaheadpoint], xmin, xmax, ymin, ymax, time_limit, v)
        if bool:
            break
    return True, startposcounter, GoalPoint, unreachable

def firstpointonpath(my_grid, robot, pathx, pathy):
    """
    Turn the robot towards the first point on the path (function only called one time per goal point)
    :param my_grid: The grid
    :param robot: To control the robot
    :param pathx, pathy: The x-coordinates and y-coordinates of the path
    :return: Nothing
    """
    psi = robot.getHeading()
    robpos = robot.getPosition()
    robposx = robpos['X']
    robposy = robpos['Y']
    if (pathy-robposy)/(pathx-robposx) < 0:
        angle = math.pi + math.atan((pathy-robposy)/(pathx-robposx))
    else:
        angle = -math.atan((pathy-robposy)/(pathx-robposx))
    robot.setMotion(0,0.8)
    i = 0
    # Turn towards first point on path, this cannot take to much time so we set a limit of 50
    while abs(angle-psi) > 0.4 and i != 50:
        time.sleep(0.1)
        psi = robot.getHeading()
        i += 1
    if i == 50:
        robot.setMotion(-v,0)
        time.sleep(0.5)
    robot.setMotion(0,0)

def purepuirsuit(my_grid, robot, pathx, pathy, xmin, xmax, ymin, ymax, time_limit, v):
    """
    Steer the robot towords the point on the path with pure puirsuit. If point to close move on to next one. Calles obstacle avoindance function.
    :params: See definition of parameters where the functions are called
    :param pathx, pathy: The x-coordinate and y-coordinate (singular!) towards which we want to steer the robot
    :return: False if point to close or when finished. True if obstacle in the way
    """
    psi = robot.getHeading()
    robpos = robot.getPosition()
    robposx = robpos['X']
    robposy = robpos['Y']
    # Change into robot coordinate system
    x = (pathx-robposx)*cos(psi)+(pathy-robposy)*sin(psi)
    y = -(pathx-robposx)*sin(psi)+(pathy-robposy)*cos(psi)
    # L is lookaheadpoint
    L = math.sqrt(x**2 + y**2)
    # If lookaheadpoint too far away move towards it for a longer time
    if L > 1.4:
        time_limit = time_limit*2
    r = L**2 / (2*y)
    omega = v/r
    robot.setMotion(v,omega)
    start_time = timer()
    now_time = timer()
    time_passed = now_time - start_time
    while time_passed < float(time_limit):
        time.sleep(0.5)
        my_grid = cartographer(my_grid)
        if obstacleavoidance(my_grid, robot, xmin, xmax, ymin, ymax):
            return True
        now_time = timer()
        time_passed = now_time - start_time
    return False

def obstacleavoidance(my_grid, robot, xmin, xmax, ymin, ymax):
    """
    Obstacle avoidance. If obstacle close back the robot up a bit. If close to edge stop.
    :param my_grid: The grid
    :param robot: To control the robot
    :param xmin, xmax, ymin, ymax: Edge coordinates of area, used to see if robot's close to the edge.
    :return: Nothing
    """
    robpos = robot.getPosition()
    robposx = robpos['X']
    robposy = robpos['Y']
    laser_scan = robot.getLaser()
    # If close to the edge
    if abs(robpos['X'] - xmin) < 0.5 or abs(robpos['X'] - xmax) < 0.5 or abs(robpos['Y'] - ymin) < 0.5 or abs(robpos['Y'] - ymax) < 0.5:
        robot.setMotion(-v, 0)
        time.sleep(2)
        robot.setMotion(0, 0)
        return True

    # If close to obstacle
    if laser_scan['Echoes'][135] < 0.3:
        if abs(robpos['X'] - xmin) < 0.5 or abs(robpos['X'] - xmax) < 0.5 or abs(robpos['Y'] - ymin) < 0.5 or abs(robpos['Y'] - ymax) < 0.5:
            robot.setMotion(-v, 0)
            time.sleep(0.3)
            robot.setMotion(0, 0)
            return True

        robot.setMotion(-v, 0)
        time.sleep(3)
        robot.setMotion(0, 0)
        return True
    return False

# Main
host = sys.argv[1]
xmin = int(sys.argv[2])
ymin = int(sys.argv[3])
xmax = int(sys.argv[4])
ymax = int(sys.argv[5])
showGUI = int(sys.argv[6])

# Test of input
if len(sys.argv) != 7:
    sys.exit("Not enough input arguments")
if xmin > xmax or ymin > ymax:
    sys.exit("You do not have the right values for the corners of the area")
if showGUI != 1 and showGUI != 0:
    sys.exit("The last input argument should be a 1 or 0")
if "http://" not in host:
    sys.exit("Not a valid URL address")
'''
# For Matilda
# host = "http://127.0.0.1:50000"
# For Jiangeng
host = "http://127.0.0.1:40000"
xmin = -30
xmax = 40
ymin = -20
ymax = 45
showGUI = 1

# Test of input
if xmin > xmax or ymin > ymax:
    sys.exit("You do not have the right values for the corners of the area")
if showGUI != 1 and showGUI != 0:
    sys.exit("The last input argument should be a 1 or 0")
if "http://" not in host:
    sys.exit("Not a valid URL address")
'''
maxVal = 15
if abs(xmax-xmin) <= 20 or abs(ymax-ymin) <= 20:
    cellsize = 0.4 # meters/grid
else:
    cellsize = 0.5
nRows = math.floor(abs(ymax - ymin) / cellsize)
nCols = math.floor(abs(xmax - xmin) / cellsize)
v = 0.4 # Velocity of robot
sleeptime = 0.5
time_limit = 1

robot = Robot(host)
my_grid = Grid(nRows, nCols)
map = ShowMap(nRows, nCols, showGUI)
curr_pos = robot.getPosition()
robot_coord = pos_to_grid(curr_pos['X'], curr_pos['Y'], xmin, ymax, cellsize)
startpos = robot_coord

# Start by turning the robot
robot.setMotion(0, 0.5)
for i in range(20):
    my_grid = cartographer(my_grid)
    time.sleep(sleeptime)
map.updateMap(my_grid.grid, maxVal, robot_coord[0], robot_coord[1])

startposcounter = 0
# boolean is true for as long as we can find new goal points
boolean = True
goalpoint = startpos
unreachable = []
while my_grid.get_nUnobserved_squares() > 0 and boolean == True:
    boolean, startposcounter ,goalpoint, unreachable = outerloop(my_grid, robot, startpos, xmin, xmax, ymin, ymax, cellsize, time_limit, v, startposcounter, goalpoint, unreachable)
# Stop the robot
robot.setMotion(0, 0)
# Close map and save latest map as map.png
map.close()
