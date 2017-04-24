# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

#grid = [[0, 1],
#        [0, 0]]
grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def findLeastCost(frontier):
    curCost, idx = min((val[0], idx) for (idx, val) in enumerate(frontier))
    costAndCell = frontier.pop(idx)
    return costAndCell[0], costAndCell[1]

def markVisited(visited, cell):
    visited[cell[0]][cell[1]] = True

def isVisited(visited, cell):
    return visited[cell[0]][cell[1]]

def move(cell, deltaVal):
    row = cell[0] + deltaVal[0]
    col = cell[1] + deltaVal[1]
    return [row, col]

def isValidCell(cell, rows, cols, grid):
    # Check for outside grid and grid not being blocked
    row = cell[0]
    col = cell[1]
    return row >= 0 and row < rows and col >= 0 and col < cols and grid[row][col] == 0

def isGoal(cell):
    return cell[0] == goal[0] and cell[1] == goal[1]

def search(grid, init, goal, cost):
    path = []
    rows = len(grid)
    cols = len(grid[0])
    visited = [[False for i in range(cols)] for j in range(rows)]
    frontier = [] # list of cost and cells
    frontier.append([0, init])

    while (len(frontier) > 0):
        curCost, cell = findLeastCost(frontier)
        
        if isGoal(cell):
            path = [curCost, cell[0], cell[1]]
            break

        if isVisited(visited, cell):
            continue

        markVisited(visited, cell)
        
        for i in range(len(delta)):
            newCell = move(cell, delta[i])
            if isValidCell(newCell, rows, cols, grid):
                newCost = curCost + cost
                frontier.append([newCost, newCell])

    if (len(path) == 0):
        path = "fail"

    return path

#print(search(grid,init,goal,cost))