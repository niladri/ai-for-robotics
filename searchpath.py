# -----------
# User Instructions:
#
# Modify the the search function so that it returns
# a shortest path as follows:
# 
# [['>', 'v', ' ', ' ', ' ', ' '],
#  [' ', '>', '>', '>', '>', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', '*']]
#
# Where '>', '<', '^', and 'v' refer to right, left, 
# up, and down motions. Note that the 'v' should be 
# lowercase. '*' should mark the goal cell.
#
# You may assume that all test cases for this function
# will have a path from init to goal.
# ----------

def printGrid(path):
    for i in range(len(path)):
        print(path[i])
        
def search(grid, init, goal, cost):
    # ----------------------------------------
    # modify code below
    # ----------------------------------------
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1
    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0
    nextExpansion = 0

    open = [[g, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand

    while not found and not resign:
        if len(open) == 0:
            resign = True
            return 'fail'
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[1]
            y = next[2]
            g = next[0]
            
            expand[x][y] = nextExpansion
            nextExpansion += 1
            
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            open.append([g2, x2, y2])
                            closed[x2][y2] = 1

    #printGrid(expand)
    path = getPath(expand, init, goal)
    return path # make sure you return the shortest path

def move(cell, deltaVal):
    row = cell[0] + deltaVal[0]
    col = cell[1] + deltaVal[1]
    return [row, col]

def isValidCell(cell, rows, cols, expand):
    # Check for outside grid and grid not being blocked
    row = cell[0]
    col = cell[1]
    return row >= 0 and row < rows and col >= 0 and col < cols and expand[row][col] >= 0

def findBestPrevious(current, expand):
    rows = len(expand)
    cols = len(expand[0])
    bestPreviousVal = expand[current[0]][current[1]]
    bestPrevious = []
    reverseDirectionIndex = 0
    for i in range(len(delta)):
        previous = move(current, delta[i])
        if (False == isValidCell(previous, rows, cols, expand)):
            continue
        if (expand[previous[0]][previous[1]] < bestPreviousVal):
            bestPrevious = previous
            bestPreviousVal = expand[previous[0]][previous[1]]
            reverseDirectionIndex = (i+2)%len(delta) # Dirty way to get reverse direction
    
    return bestPrevious, reverseDirectionIndex
    
def getPath(expand, init, goal):
    path = [[' ' for col in range(len(expand[0]))] for row in range(len(expand))]
    current = goal
    path[goal[0]][goal[1]] = '*'

    while (current != init):
        previous, directionIndex = findBestPrevious(current, expand)
        path[previous[0]][previous[1]] = delta_name[directionIndex]
        current = previous

    return path

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

#printGrid(search(grid, init, goal, cost))