# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess 
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered 
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position. 
#
# ----------
# GRADING
# 
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *
from math import *
from matrix import *
import random
import time

# Does not work completely

class trackstate:
    def __init__(self, x, P, cnt = 0):
        self.x = x
        self.P = P
        self.count = cnt
        self.measurement = []
        self.estimate = []

# Using Kalman 5D
# Using state vector [x, y, theta, dtheta, d]

def predict(x, P):
    # prediction

    # pull out current estimates based on measurement
    # this was a big part of what was hainging me up (I was using older estimates before)
    x0 = x.value[0][0]
    y0 = x.value[1][0]
    theta0 = x.value[2][0]
    dtheta0 = x.value[3][0]
    dist0 = x.value[4][0]

    # next state function: 
    # this is now the Jacobian of the transition matrix (F) from the regular Kalman Filter
    A =  matrix([[1.0, 0.0, -dist0*sin(theta0+dtheta0), -dist0*sin(theta0+dtheta0), cos(theta0+dtheta0)],
                 [0.0, 1.0, dist0*cos(theta0+dtheta0), dist0*cos(theta0+dtheta0), sin(theta0+dtheta0)],
                 [0.0, 0.0, 1.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 0.0, 1.0]])

    # calculate new estimate 
    # it's NOT simply the matrix multiplication of transition matrix and estimated state vector
    # for the EKF just use the state transition formulas the transition matrix was built from
    x = matrix([[x0 + dist0 * cos(theta0 + dtheta0)],
                [y0 + dist0 * sin(theta0 + dtheta0)],
                [angle_trunc(theta0 + dtheta0)],
                [dtheta0], 
                [dist0]])

    P = A * P * A.transpose() 
    
    return x, P

def kalman_filter(x, P, measurement):
    
    # measurement function: reflect the fact that we observe x and y 
    H =  matrix([[1.0, 0.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0, 0.0]])
    
    # measurement uncertainty: use 2x2 matrix with 0.05 as main diagonal
    R =  matrix([[measurement_noise, 0.0], [0.0, measurement_noise]])    

    dim = x.dimx
    I = matrix([[]])
    I.identity(dim)

    # measurement update
    Z = matrix([measurement])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P   

    x, P = predict(x, P)

    return x, P

# This is the function you have to write. The argument 'measurement' is a 
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be 
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    x = matrix([[0.0], [0.0], [0.0], [0.0], [0.0]]) # initial state (x, y, theta, dtheta, d)      
    P =  matrix([[1000.0, 0.0, 0.0, 0.0, 0.0], 
                 [0.0, 1000.0, 0.0, 0.0, 0.0], 
                 [0.0, 0.0, 1000.0, 0.0, 0.0], 
                 [0.0, 0.0, 0.0, 1000.0, 0.0], 
                 [0.0, 0.0, 0.0, 0.0, 1000.0]]) 
    # initial uncertainty: 1000 for each state variable

    xy_estimate = []
    state = OTHER
    if not OTHER: # this is the first measurement  
        # using initial guesses
        x, P = kalman_filter(x, P, measurement)
        state = trackstate(x, P)
        xy_estimate = measurement
    else:
        # measure and then predict
        x, P = kalman_filter(state.x, state.P, measurement)
        state = trackstate(x, P, OTHER.count+1)
        xy_estimate = x.value[0][0], x.value[1][0]      
    
    OTHER = state
    OTHER.measurement = measurement
    OTHER.estimate = xy_estimate
    return xy_estimate, OTHER    

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    if OTHER:
        print("error in previous estimate: ", distance_between(OTHER.estimate, target_measurement))
    
    xy_estimate, OTHER = estimate_next_pos(target_measurement, OTHER)
    distance = distance_between(hunter_position, xy_estimate)
    
    # If distance > max_distance, plan for few steps ahead
    if distance > max_distance and OTHER.count > 5:
        oldsteps = 0
        steps = int(ceil(distance/max_distance))
        print("steps required: ", steps, "hunter: ",  '[%.3f, %.3f]' % (hunter_position[0], hunter_position[1]))
        
        intercept_steps = 0
        # Find interception point if esists
        for k in range(1, steps):
            step_n_estimate = get_step_n(OTHER, k)
            distance_to_step_n = distance_between(hunter_position, step_n_estimate)
            if distance_to_step_n < distance:
                # Check if intercept possible
                steps_required_by_hunter = int(ceil(distance_to_step_n/max_distance))
                if steps_required_by_hunter <= k:
                    #print("intercept possible in ", k)
                    intercept_steps = k
                    distance = distance_to_step_n
                    xy_estimate = step_n_estimate
                    break
           
        # Intercept not found - find the next quickest intercept
        if intercept_steps == 0:
            #print("intercept not found")
            while True:
                step_n_estimate = get_step_n(OTHER, steps)
                #print("steps: ", steps, 'estimate [%.3f, %.3f]' % (xy_estimate[0], xy_estimate[1]))
                distance_to_step_n = distance_between(hunter_position, step_n_estimate)
                steps_required_by_hunter = int(ceil(distance_to_step_n/max_distance))
                if steps_required_by_hunter <= steps:
                    #print("future intercept possible in ", steps_required_by_hunter)
                    distance = distance_to_step_n
                    xy_estimate = step_n_estimate
                    break
                steps += 1
    
    distance = min(distance, max_distance)
    turning = angle_trunc(get_heading(hunter_position, xy_estimate) - hunter_heading)
    print('distance: %.3f, turning: %.3f, [%.3f, %.3f]' % (distance, turning, xy_estimate[0], xy_estimate[1]))
    
    #time.sleep(2)
        
    return turning, distance, OTHER

def get_step_n(OTHER, n):
    x = OTHER.x
    P = OTHER.P
    for i in range(n-1):
        x, P = predict(x, P)
    
    return x.value[0][0], x.value[1][0]

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any 
# information that you want. 
def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print("You got it right! It took you ", ctr, " steps to catch the target.")
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)
        
        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1            
        if ctr >= 1000:
            print("It took too many steps to catch the target.")
    return caught

def demo_grading_v(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print("You got it right! It took you ", ctr, " steps to catch the target.")
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        #Visualize it
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        #End of visualization
        ctr += 1            
        if ctr >= 1000:
            print("It took too many steps to catch the target.")
    return caught

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
target = robot(0.0, 10.0, 0.0, 2*pi / 5, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)
hunter = robot(2.0, 10.0, 0.0)

print(demo_grading_v(hunter, target, next_move))