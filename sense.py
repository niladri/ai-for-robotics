#Modify the code so that it updates the probability twice
#and gives the posterior distribution after both 
#measurements are incorporated. Make sure that your code 
#allows for any sequence of measurement of any length.

p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
pHit = 0.6
pMiss = 0.2

def sense(p, Z):
    q=[]
    for i in range(len(p)):
        pZ = 0.6 if world[i] == Z else 0.2
        q.append(p[i]*pZ)
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q

def msense(p, Z):
    for i in range(len(measurements)):
        p = sense(p, measurements[i])
        #print p
    return p

print msense(p, measurements)
