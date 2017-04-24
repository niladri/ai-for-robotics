# ----------
# User Instructions:
# 
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal. 
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

valMax = 99

def printGrid(mat):
    for i in range(len(mat)):
        print mat[i]
        
def calculateValue(cost, valueTable, row, col, rows, cols, visited, grid):
    if (valueTable[row][col] < valMax):
        # Value is already calculated
        return
        
    minVal = valMax
    # Find minimum
    for i in range(len(delta)):
        newRow = row + delta[i][0]
        newCol = col + delta[i][1]
        
        if ((newRow >= 0 and newRow < rows and newCol >= 0 and newCol < cols) == False):
            continue
        
        if (grid[newRow][newCol] == 1):
            # This grid cell is an obstacle
            continue
        
        if (visited[newRow][newCol] == 1):
            continue

        visited[newRow][newCol] = 1
        calculateValue(cost, valueTable, newRow, newCol, rows, cols, visited, grid)
        visited[newRow][newCol] = 0

        val = valueTable[newRow][newCol]
        if (val < minVal):
            minVal = val
    
    if (minVal == valMax):
        # Could not find suitable source of already calculated value
        return
    
    valueTable[row][col] = cost + minVal
            
def compute_value(grid, goal, cost):
    rows = len(grid)
    cols = len(grid[0])
    valueTable = [[-1 for col in range(cols)] for row in range(rows)]
    for row in range(rows):
        for col in range(cols):
                valueTable[row][col] = valMax
    valueTable[goal[0]][goal[1]] = 0
      
    #printGrid(valueTable) 
    visited = [[0 for col in range(cols)] for row in range(rows)]
    for row in range(rows):
        for col in range(cols):
            if (grid[row][col] == 0):
                calculateValue(cost, valueTable, row, col, rows, cols, visited, grid)
    return valueTable 

#printGrid(compute_value(grid, goal, cost))