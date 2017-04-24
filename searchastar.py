# -----------
# User Instructions:
#
# Modify the the search function so that it becomes
# an A* search algorithm as defined in the previous
# lectures.
#
# Your function should return the expanded grid
# which shows, for each element, the count when
# it was expanded or -1 if the element was never expanded.
# 
# If there is no path from init to goal,
# the function should return the string 'fail'
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def printGrid(mat):
    for i in range(len(mat)):
        print mat[i]

def search(grid, init, goal, cost, heuristic):
    # ----------------------------------------
    # modify the code below
    # ----------------------------------------
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    row = init[0]
    col = init[1]
    g = 0
    h = heuristic[row][col]
    f = g + h

    open = [[f, g, h, row, col]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
    
    while not found and not resign:
        if len(open) == 0:
            resign = True
            return "Fail"
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            row = next[3]
            col = next[4]
            g = next[1]
            expand[row][col] = count
            count += 1
            
            if row == goal[0] and col == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    newRow = row + delta[i][0]
                    newCol = col + delta[i][1]
                    if newRow >= 0 and newRow < len(grid) and newCol >=0 and newCol < len(grid[0]):
                        if closed[newRow][newCol] == 0 and grid[newRow][newCol] == 0:
                            g2 = g + cost
                            h2 = heuristic[newRow][newCol]
                            f2 = g2 + h2
                            open.append([f2, g2, h2, newRow, newCol])
                            closed[newRow][newCol] = 1

    return expand

printGrid(search(grid, init, goal, cost, heuristic))