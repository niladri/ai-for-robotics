# ----------
# User Instructions:
# 
# Write a function optimum_policy that returns
# a grid which shows the optimum policy for robot
# motion. This means there should be an optimum
# direction associated with each navigable cell from
# which the goal can be reached.
# 
# Unnavigable cells as well as cells from which 
# the goal cannot be reached should have a string 
# containing a single space (' '), as shown in the 
# previous video. The goal cell should have '*'.
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

maxVal = 99

def printGrid(mat):
    for i in range(len(mat)):
        print mat[i]
        
def optimum_policy(grid, goal, cost):
    # ----------------------------------------
    # modify code below
    # ----------------------------------------
    value = [[maxVal for row in range(len(grid[0]))] for col in range(len(grid))]
    change = True

    while change:
        change = False

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                if goal[0] == x and goal[1] == y:
                    if value[x][y] > 0:
                        value[x][y] = 0

                        change = True

                elif grid[x][y] == 0:
                    for a in range(len(delta)):
                        x2 = x + delta[a][0]
                        y2 = y + delta[a][1]

                        if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                            v2 = value[x2][y2] + cost

                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2

    #printGrid(value)
    policy = getPaths(value, goal)
    return policy

def move(cell, deltaVal):
    row = cell[0] + deltaVal[0]
    col = cell[1] + deltaVal[1]
    return [row, col]

def isValidCell(cell, rows, cols, vt):
    # Check for outside grid and grid not being blocked
    row = cell[0]
    col = cell[1]
    return row >= 0 and row < rows and col >= 0 and col < cols and vt[row][col] < maxVal

def findNext(current, vt):
    rows = len(vt)
    cols = len(vt[0])
    bestNextVal = vt[current[0]][current[1]]
    directionIndex = 0
    for i in range(len(delta)):
        next = move(current, delta[i])
        if (False == isValidCell(next, rows, cols, vt)):
            continue
        if (vt[next[0]][next[1]] < bestNextVal):
            bestNextVal = vt[next[0]][next[1]]
            directionIndex = i
    
    return directionIndex
    
def getPaths(valueTable, goal):
    path = [[' ' for col in range(len(valueTable[0]))] for row in range(len(valueTable))]
    path[goal[0]][goal[1]] = '*'

    for row in range(len(valueTable)):
        for col in range(len(valueTable[0])):
            if (row == goal[0] and col == goal[1]):
                continue
            if (valueTable[row][col] == maxVal):
                continue
            directionIndex = findNext([row, col], valueTable)
            path[row][col] = delta_name[directionIndex]

    return path
    
#printGrid(optimum_policy(grid, goal, cost))