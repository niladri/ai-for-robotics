# -----------
# User Instructions
#
# Define a function smooth that takes a path as its input
# (with optional parameters for weight_data, weight_smooth,
# and tolerance) and returns a smooth path. The first and 
# last points should remain unchanged.
#
# Smoothing should be implemented by iteratively updating
# each entry in newpath until some desired level of accuracy
# is reached. The update should be done according to the
# gradient descent equations given in the instructor's note
# below (the equations given in the video are not quite 
# correct).
# -----------

from copy import deepcopy

# thank you to EnTerr for posting this on our discussion forum
def printpaths(path,newpath):
    for old,new in zip(path,newpath):
        print '['+ ', '.join('%.3f'%x for x in old) + \
               '] -> ['+ ', '.join('%.3f'%x for x in new) +']'

# Don't modify path inside your function.
path = [[0, 0],
        [0, 1],
        [0, 2],
        [1, 2],
        [2, 2],
        [3, 2],
        [4, 2],
        [4, 3],
        [4, 4]]

def applysmooth(xi, yi, yp, yn, alpha, beta):
    #yi = yi + alpha (xi - yi) + beta (yi + 1 + yi - 1 - 2 * yi)
    yi = yi + (alpha * (xi - yi)) + (beta * (yp + yn - (2 * yi)))
    return yi
    
def smooth(path, weight_data = 0.5, weight_smooth = 0.1, tolerance = 0.000001):

    # Make a deep copy of path into newpath
    newpath = deepcopy(path)

    change = 1.0
    while change > tolerance:
        change = 0.0
        for i in range(1, len(path)-1, 1):
            newx = applysmooth(path[i][0], newpath[i][0], newpath[i+1][0], newpath[i-1][0], 
                weight_data, weight_smooth)
            newy = applysmooth(path[i][1], newpath[i][1], newpath[i+1][1], newpath[i-1][1], 
                weight_data, weight_smooth)
            dx = abs(newx - newpath[i][0])
            dy = abs(newy - newpath[i][1])
            if (dx + dy > change):
                change = dx + dy
            newpath[i][0] = newx
            newpath[i][1] = newy
                
    return newpath # Leave this line for the grader!

#printpaths(path,smooth(path))
