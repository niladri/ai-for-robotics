# ----------
# User Instructions:
# 
# Implement the function optimum_policy2D below.
#
# You are given a car in grid with initial state
# init. Your task is to compute and return the car's 
# optimal path to the position specified in goal; 
# the costs for each motion are as defined in cost.
#
# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a 
# right turn.

forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

# EXAMPLE INPUTS:
# grid format:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0] # given in the form [row,col,direction]
                 # direction = 0: up
                 #             1: left
                 #             2: down
                 #             3: right
                
goal = [2, 0] # given in the form [row,col]

#grid = [[0, 0],
#        [0, 0]]
#init = [1, 1, 0]
#goal = [0, 0]

cost = [2, 1, 20] # cost has 3 values, corresponding to making 
                  # a right turn, no turn, and a left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return 
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]
# ----------

# ----------------------------------------
# modify code below
# ----------------------------------------

def printGrid(mat):
    for i in range(len(mat)):
        print(mat[i])
        
def optimum_policy2D(grid, init, goal, cost):
    maxVal = 999
    valueTable = [[[maxVal for d in range(len(forward))] for col in range(len(grid[0]))] for row in range(len(grid))]
    change = True
    
    rows = len(grid)
    cols = len(grid[0])

    while change:
        change = False

        for row in range(len(grid)):
            for col in range(len(grid[0])):
                for orientation in range(len(forward)):
                    if goal[0] == row and goal[1] == col:
                        if valueTable[row][col][orientation] > 0:
                            valueTable[row][col][orientation] = 0

                            change = True

                    elif grid[row][col] == 0:
                        for a in range(len(action)):
                            newOrientation = (orientation + action[a]) % len(forward)
                            moveStep = forward[newOrientation]
                            newRow = row + moveStep[0]
                            newCol = col + moveStep[1]

                            if newRow >= 0 and newRow < rows and newCol >= 0 and newCol < cols and grid[newRow][newCol] == 0:
                                newVal = valueTable[newRow][newCol][newOrientation] + cost[a]
                                if newVal < valueTable[row][col][orientation]:
                                    change = True
                                    valueTable[row][col][orientation] = newVal

    path = [[' ' for col in range(len(valueTable[0]))] for row in range(len(valueTable))]
    current = init
    maxVal = 999
    
    rows = len(valueTable)
    cols = len(valueTable[0])
    orientations = len(valueTable[0][0])
    
    # If path does not exist
    if (valueTable[current[0]][current[1]][current[2]] == maxVal):
        return path
    
    path[goal[0]][goal[1]] = '*'
    while (current[0] != goal[0] or current[1] != goal[1]):
        row = current[0]
        col = current[1]
        orientation = current[2]

        nextState = []
        actionIndex = -1

        for i in range(len(action)):
            newOrientation = (orientation + action[i]) % len(forward)
            moveStep = forward[newOrientation]
            newRow = row + moveStep[0]
            newCol = col + moveStep[1]
            if (False == (newRow >= 0 and newRow < rows and newCol >= 0 and newCol < cols)):
                continue
            # If action cost with current cost matches then it is an action in the path
            currentCost = valueTable[row][col][orientation]
            actionCost = cost[i]
            newCost = valueTable[newRow][newCol][newOrientation]
            if (currentCost - actionCost == newCost):
                nextState = [newRow, newCol, newOrientation]
                actionIndex = i
        path[current[0]][current[1]] = action_name[actionIndex]
        current = nextState

    return path
    
#printGrid(optimum_policy2D(grid, init, goal, cost))