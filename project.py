# ----------
# Background
# 
# A robotics company named Trax has created a line of small self-driving robots 
# designed to autonomously traverse desert environments in search of undiscovered
# water deposits.
#
# A Traxbot looks like a small tank. Each one is about half a meter long and drives
# on two continuous metal tracks. In order to maneuver itself, a Traxbot can do one
# of two things: it can drive in a straight line or it can turn. So to make a 
# right turn, A Traxbot will drive forward, stop, turn 90 degrees, then continue
# driving straight.
#
# This series of questions involves the recovery of a rogue Traxbot. This bot has 
# gotten lost somewhere in the desert and is now stuck driving in an almost-circle: it has
# been repeatedly driving forward by some step size, stopping, turning a certain 
# amount, and repeating this process... Luckily, the Traxbot is still sending all
# of its sensor data back to headquarters.
#
# In this project, we will start with a simple version of this problem and 
# gradually add complexity. By the end, you will have a fully articulated
# plan for recovering the lost Traxbot.
# 
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

# Using Kalman 6D
F =  matrix([[1.0, 0.0, 1.0, 0.0, 0.5, 0.0, 0.0, 0.0], 
             [0.0, 1.0, 0.0, 1.0, 0.0, 0.5, 0.0, 0.0],
             [0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.5, 0.0],
             [0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.5],
             [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0], 
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], 
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
# covariance matrix
q = F * F.transpose() * matrix([[0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
             [0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
             [0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0],
             [0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0],
             [0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0],
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0], 
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0], 
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]])
# next state function

H =  matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
# measurement function: reflect the fact that we observe x and y but not the two velocities

R =  matrix([[0.05, 0.0], [0.0, 0.05]])
# measurement uncertainty: use 2x2 matrix with 0.05 as main diagonal

def kalman_filter(x, P, measurement):
    
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
    
    # prediction
    x = F * x
    P = F * P * F.transpose() + q
    
    #print('x= ')
    #x.show()
    #print('P= ')
    #P.show()
    
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

    x = matrix([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]) # initial state (location, velocity and acceleration)      
    P =  matrix([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                 [0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                 [0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 0.0], 
                 [0.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0], 
                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0, 0.0], 
                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0]]) 
    # initial uncertainty: 0 for positions x and y, 1000 for the two velocities
    
    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes. 
    xy_estimate = []
    orientation_estimate = 0.0
    state = []
    if not OTHER: # this is the first measurement  
        # only prediction
        #x = F * x
        #P = F * P * F.transpose() 
        #OTHER = []
        #OTHER.append(x)
        #OTHER.append(P)
        #xy_estimate = measurement
        xy_estimate = measurement
        orientation_estimate = atan2(measurement[1], measurement[0])
        state.append(measurement)
        state.append(orientation_estimate)
        state.append(0) # count
    else:
        # measure and then predict
        #x, P = kalman_filter(OTHER[0], OTHER[1], measurement)
        #OTHER[0] = x
        #OTHER[1] = P
        #xy_estimate = x.value[0][0], x.value[1][0]
        previous_measurement = OTHER[0]
        dx = measurement[0] - previous_measurement[0]
        dy = measurement[1] - previous_measurement[1]
        orientation_current = atan2(dy, dx)
        orientation_previous = OTHER[1]
        #print("c: ", measurement, "p: ", previous_measurement)
        #print("dx: ", dx, "dy: ", dy)
        #print("o_c:", orientation_current, "o_p: ", orientation_previous)
        dtheta = orientation_current - orientation_previous
        if dtheta < -pi/2:
            dtheta += 2 * pi
        d = distance_between(measurement, previous_measurement)
        #print("d: ", d, "dtheta: ", dtheta)
        orientation_estimate = orientation_current + dtheta
        count = OTHER[2]
        if count == 0:
            dList = [d]
            wList = [dtheta]
            x_estimate = measurement[0] + d * cos(orientation_estimate)
            y_estimate = measurement[1] + d * sin(orientation_estimate)
            xy_estimate = x_estimate, y_estimate
            state.append(measurement)
            state.append(orientation_current)  
            state.append(count+1)
            state.append(dList)
            state.append(wList)            
        else:
            dList = OTHER[3]
            wList = OTHER[4]
            if not len(dList) == len(wList):
                raise(ValueError, "List of past d and theta should have equal lengths")
            d_weighted = (sum(dList) + d) / (len(dList) + 1)
            dtheta_weighted = (sum(wList) + dtheta) / (len(wList) + 1)
            #print("d_w:", d_weighted, "t_w:", dtheta_weighted)
            orientation_estimate = orientation_current + dtheta_weighted
            x_estimate = measurement[0] + d_weighted * cos(orientation_estimate)
            y_estimate = measurement[1] + d_weighted * sin(orientation_estimate)
            xy_estimate = x_estimate, y_estimate
            dList.append(d)
            wList.append(dtheta)
            state.append(measurement)
            state.append(orientation_current)  
            state.append(count+1)
            state.append(dList)
            state.append(wList)             
    OTHER = state
    return xy_estimate, OTHER    

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any 
# information that you want. 
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        print("error: ", error, " tolerance: ", distance_tolerance)
        if error <= distance_tolerance:
            print("You got it right! It took you ", ctr, " steps to localize.")
            localized = True
        if ctr == 1000:
            print("Sorry, it took you too many steps to localize the target.")
    return localized

def demo_grading_v(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization
    import turtle    #You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.bgcolor('white')
    size_multiplier= 25.0  #change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.1, 0.1, 0.1)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('circle')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(0.1, 0.1, 0.1)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.1, 0.1, 0.1)
    prediction.penup()
    broken_robot.penup()
    measured_broken_robot.penup()
    #End of Visualization
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        #print("error: ", error, " tolerance: ", distance_tolerance)
        if error <= distance_tolerance:
            print("You got it right! It took you ", ctr, " steps to localize.")
            localized = True
        if ctr == 1000:
            print("Sorry, it took you too many steps to localize the target.")
        #More Visualization
        measured_broken_robot.setheading(target_bot.heading*180/pi)
        measured_broken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-200)
        measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(position_guess[0]*size_multiplier, position_guess[1]*size_multiplier-200)
        prediction.stamp()
        #End of Visualization
    return localized

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that 
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER 
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(estimate_next_pos, test_target)
