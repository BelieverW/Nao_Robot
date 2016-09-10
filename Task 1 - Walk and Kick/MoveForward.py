# MoveForward.py
import sys
import time
from naoqi import ALProxy
from math import pi,sin,cos,asin,acos,atan2,sqrt

# The length of Thigh
ThighLength = 100.0
# The length of Tibia
TibiaLength = 102.90

HipHeight = -200
HipOffsetY = 65
AnkleHeight = 75
StepLength = 20
dh = 30
# getAngles(x, y, z) is the function solve the inverse kinetic problem
# @return the angle list "angle" containing five values of leg's angles
# @param (x, y, z) represents the coordinate of ankle based on hip
def getAngles(x, y, z):
    # initialize the angle list
    # angle[0] is HipRoll
    # angle[1] is HipPitch
    # angle[2] is KneePitch
    # angle[3] is AnklePitch
    # angle[4] is AnkleRoll
    angle = [0, 0, 0, 0, 0]

    angle[0] = atan2(y, -z)
    k = z * cos(angle[0]) - y * sin(angle[0])
    tmp = x * x + y * y + z * z - ThighLength * ThighLength - TibiaLength * TibiaLength
    print tmp / (2 * TibiaLength * ThighLength)
    angle[2] = acos(tmp / (2 * TibiaLength * ThighLength))
    m = (x * x + k * k + ThighLength * ThighLength - TibiaLength * TibiaLength)/(2 * k)
    a = 2 * x * m / k;
    tmp = x * x / (k * k) - 1
    b = a * a - 4 * (ThighLength * ThighLength - m * m) * tmp
    c = 2 * ThighLength * tmp
    d = (a + sqrt(b)) / c
    if abs(d) > 1:
        d = (a - sqrt(b)) / c 
    angle[1] = abs(asin(d))
    angle[1] = -angle[1]
    angle[3] = -angle[1] - angle[2]
    angle[4] = -angle[0]
    return angle

def firstStep():
    names = ["RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll"]
    #motionProxy.setStiffnesses(names, 1.0)
    angleLists  = [range(5) for i in range(len(names))]

    # squat
    i = 0
    angRight = getAngles(0, 0, HipHeight + dh)
    angLeft  = getAngles(0, 0, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    # move center of gravity to the center of right leg
    i = 1
    angRight = getAngles(0, HipOffsetY, HipHeight + dh)
    angLeft  = getAngles(0, HipOffsetY, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    # lift the left leg
    i = 2
    angRight = getAngles(0, HipOffsetY, HipHeight + dh)
    angLeft  = getAngles(0, HipOffsetY, HipHeight + AnkleHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    # move center of gravity
    i = 3
    angRight = getAngles(0, HipOffsetY, HipHeight + dh)
    angLeft  = getAngles(2 * StepLength, HipOffsetY, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    i = 4
    angRight = getAngles(-StepLength, 0, HipHeight + dh)
    angLeft  = getAngles(StepLength, 0, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    timeLists = [[1, 2, 3, 4, 5] for j in range(len(names))]
    print timeLists
    print angleLists
    motionProxy.angleInterpolation(names, angleLists, timeLists, True)

def lastStep():
    names = ["RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll"]
    # angleLists  = [range(3) for i in range(len(names))]
    # timeLists = [[1, 1.8, 2] for j in range(len(names))]
    angleLists  = [range(4) for i in range(len(names))]
    timeLists = [[1, 2,3,4] for j in range(len(names))]

    i = 0
    angLeft = getAngles(0, -HipOffsetY+10, HipHeight + dh)
    angRight  = getAngles(-2 * StepLength, -HipOffsetY+10, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    i = 1
    angRight = getAngles(0, -HipOffsetY+10, HipHeight + dh + 30)
    angLeft  = getAngles(0, -HipOffsetY+10, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    i = 2
    angRight = getAngles(0, -HipOffsetY+10, HipHeight + dh)
    angLeft  = getAngles(0, -HipOffsetY+10, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]


    i = 3
    angRight = getAngles(0, 0, HipHeight + dh)
    angLeft  = getAngles(0, 0, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    motionProxy.angleInterpolation(names, angleLists, timeLists, True)

def kick():
    names = ["RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll"]
    # angleLists  = [range(3) for i in range(len(names))]
    # timeLists = [[1, 1.8, 2] for j in range(len(names))]
    angleLists  = [range(5) for i in range(len(names))]
    timeLists = [[1, 2, 2.15, 4, 5] for j in range(len(names))]

    i = 0
    angLeft = getAngles(0, -HipOffsetY+10, HipHeight + dh)
    angRight  = getAngles(-2 * StepLength, -HipOffsetY+10, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    i = 1
    angRight = getAngles(0, -HipOffsetY+10, HipHeight + dh + 10)
    angLeft  = getAngles(0, -HipOffsetY+10, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    i = 2
    angRight = getAngles(50, -HipOffsetY+10, HipHeight + dh + 20 )
    angLeft  = getAngles(0, -HipOffsetY+10, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    i = 3
    angRight = getAngles(0, -HipOffsetY+10, HipHeight + dh + 10)
    angLeft  = getAngles(0, -HipOffsetY+10, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    i = 4
    angRight = getAngles(0, 0, HipHeight + dh)
    angLeft  = getAngles(0, 0, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    motionProxy.angleInterpolation(names, angleLists, timeLists, True)

def moveForwardAndKick(step):
    names = ["RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll"]
    #step = step / 2
    x = []
    z = []
    timeStepList = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    for t in timeStepList:
        temp = -104.7 + 148.3 * t - 1647.9 * t * t + 7202.1 * t * t * t - 9155.3 * t * t * t * t + 3662.1 *t * t * t * t * t
        if temp <= 0:
            temp += 70
        if temp > 0:
            temp -= 70
        x = x + [temp]
        z = z + [30 + 177.9 * t - 2155.5 * t * t + 11279 * t * t * t - 23950 * t * t * t * t + 21973 *t * t * t * t * t - 7324.2 *t * t * t * t * t * t]

    motionProxy.setStiffnesses(names, 1.0)
    firstStep()
    '''
    for k in range(step):
        angleLists  = [range(len(timeStepList)) for i in range(len(names))]
        angleLists2  = [range(len(timeStepList)) for i in range(len(names))]
        timeLists = [[1, 2, 3, 4, 5, 6, 7, 8, 9, 10] for j in range(len(names))]

        for i in range(len(timeStepList)):
            angRight = getAngles(x[i], 0, HipHeight + z[i])
            angLeft  = getAngles(-x[i], 0, HipHeight + dh)
            for j in range(5):
                angleLists[j][i] = angRight[j]
            for j in range(5,10):
                angleLists[j][i] = angLeft[j - 5]
        motionProxy.angleInterpolation(names, angleLists, timeLists, True)
        for i in range(len(timeStepList)):
            angLeft  = getAngles(x[i], 0, HipHeight + z[i])
            angRight = getAngles(-x[i], 0, HipHeight + dh)
            for j in range(5):
                angleLists2[j][i] = angRight[j]
            for j in range(5,10):
                angleLists2[j][i] = angLeft[j - 5]
        motionProxy.angleInterpolation(names, angleLists2, timeLists, True)
    '''
    #twoStep()
    # 16 *
    for i in range(step):
        twoStep()
    kick()
#    lastStep()
    print "finished"

def moveForward(step):
    names = ["RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll"]
    #step = step / 2
    x = []
    z = []
    timeStepList = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    for t in timeStepList:
        temp = -104.7 + 148.3 * t - 1647.9 * t * t + 7202.1 * t * t * t - 9155.3 * t * t * t * t + 3662.1 *t * t * t * t * t
        if temp <= 0:
            temp += 70
        if temp > 0:
            temp -= 70
        x = x + [temp]
        z = z + [30 + 177.9 * t - 2155.5 * t * t + 11279 * t * t * t - 23950 * t * t * t * t + 21973 *t * t * t * t * t - 7324.2 *t * t * t * t * t * t]

    motionProxy.setStiffnesses(names, 1.0)
    firstStep()
    '''
    for k in range(step):
        angleLists  = [range(len(timeStepList)) for i in range(len(names))]
        angleLists2  = [range(len(timeStepList)) for i in range(len(names))]
        timeLists = [[1, 2, 3, 4, 5, 6, 7, 8, 9, 10] for j in range(len(names))]

        for i in range(len(timeStepList)):
            angRight = getAngles(x[i], 0, HipHeight + z[i])
            angLeft  = getAngles(-x[i], 0, HipHeight + dh)
            for j in range(5):
                angleLists[j][i] = angRight[j]
            for j in range(5,10):
                angleLists[j][i] = angLeft[j - 5]
        motionProxy.angleInterpolation(names, angleLists, timeLists, True)
        for i in range(len(timeStepList)):
            angLeft  = getAngles(x[i], 0, HipHeight + z[i])
            angRight = getAngles(-x[i], 0, HipHeight + dh)
            for j in range(5):
                angleLists2[j][i] = angRight[j]
            for j in range(5,10):
                angleLists2[j][i] = angLeft[j - 5]
        motionProxy.angleInterpolation(names, angleLists2, timeLists, True)
    '''
    #twoStep()
    # 16 *
    for i in range(step):
        twoStep()
#    kick()
    lastStep()
    print "finished"
'''
def twoStep():
    names = ["RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll"]
    #motionProxy.setStiffnesses(names, 1.0)
    angleLists  = [range(6) for i in range(len(names))]
    timeLists = [[3, 6, 8, 10, 12, 14] for j in range(len(names))]

    i = 0
    angLeft = getAngles(0, -HipOffsetY, HipHeight + dh)
    angRight  = getAngles(-2 * StepLength, -HipOffsetY, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    i = 1
    angLeft = getAngles(0, -HipOffsetY, HipHeight + dh)
    angRight  = getAngles(0, -HipOffsetY, HipHeight + dh + AnkleHeight)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    i = 2
    angLeft = getAngles(0, -HipOffsetY, HipHeight + dh)
    angRight  = getAngles(2 * StepLength, -HipOffsetY, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    i = 3
    angLeft = getAngles(-2 * StepLength, HipOffsetY, HipHeight + dh)
    angRight  = getAngles(0, HipOffsetY, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    i = 4
    angLeft = getAngles(0, HipOffsetY, HipHeight + dh + AnkleHeight)
    angRight  = getAngles(0, HipOffsetY, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    i = 5
    angLeft = getAngles(2 * StepLength, HipOffsetY, HipHeight + dh)
    angRight  = getAngles(0, HipOffsetY, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    motionProxy.angleInterpolation(names, angleLists, timeLists, True)
'''

def twoStep():
    timeStepList = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    # try to change timeStepList to timeStepList = [0, 0.2, 0.4, 0.6, 0.8, 1.0]
    names = ["RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll"]
        #motionProxy.setStiffnesses(names, 1.0)
    angleLists  = [range(2 * len(timeStepList) + 2) for i in range(len(names))]
    #print len(range(2 * len(timeStepList) + 1))
    time = range(1, 2 * len(timeStepList) + 3)
    for i in range(len(time)):
        time[i] = time[i] /5.0
    print time
    print len(time)
    print len(time)/2
    for i in range(11, len(time)):
        time[i] +=1
    for i in range(12, len(time)):
        time[i] +=1
    time[len(time)-1] += 1
    for i in range(len(time)):
        time[i] += 1
    print time
    time = [1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 4.0, 5.0, 5.8, 6.0, 6.2, 6.4, 6.6, 6.8, 7.0, 7.2, 7.4, 7.6, 8.8]
    timeLists = [time for j in range(len(names))]
    print timeLists

    #print timeLists[0][:]
    #print len(timeLists[0])
    x = []
    z = []
    for t in timeStepList:
        tempX = -104.7 + 148.3 * t - 1647.9 * t * t + 7202.1 * t * t * t - 9155.3 * t * t * t * t + 3662.1 *t * t * t * t * t
        tempX = tempX / 104.7 * StepLength
        tempZ = 177.9 * t - 2155.5 * t * t + 11279 * t * t * t - 23950 * t * t * t * t + 21973 *t * t * t * t * t - 7324.2 *t * t * t * t * t * t
        tempZ = tempZ/35*50 #/ 30.0 * 40 # ?45
    #if temp <= 0 :
        #temp += 70
    #if temp > 0:
    #    temp += -70
        x = x + [tempX]
        z = z + [tempZ]

    i = 0
    angLeft = getAngles(0, -HipOffsetY, HipHeight + dh)
    angRight  = getAngles(-2 * StepLength, -HipOffsetY, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    for i in range(1, len(timeStepList)):
        angLeft = getAngles(0, -HipOffsetY, HipHeight + dh)
        angRight  = getAngles(2 * x[i], -HipOffsetY, HipHeight + dh + z[i])
        for j in range(5):
            angleLists[j][i] = angRight[j]
        for j in range(5,10):
            angleLists[j][i] = angLeft[j - 5]

    i = len(timeStepList)
    angLeft = getAngles(-1 * StepLength, 0, HipHeight + dh)
    angRight  = getAngles(StepLength, 0, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    for i in range(len(timeStepList) + 1, 2 * len(timeStepList) + 1):
        angLeft = getAngles(2 * x[i - len(timeStepList)-1], HipOffsetY, HipHeight + dh + z[i - len(timeStepList)-1])
        angRight  = getAngles(-0, HipOffsetY, HipHeight + dh)
        for j in range(5):
            angleLists[j][i] = angRight[j]
        for j in range(5,10):
            angleLists[j][i] = angLeft[j - 5]

    i = 2 * len(timeStepList) + 1
    angLeft = getAngles(StepLength, 0, HipHeight + dh)
    angRight  = getAngles(-StepLength, 0, HipHeight + dh)
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]
    print len(angleLists[0])
    motionProxy.angleInterpolation(names, angleLists, timeLists, True)

    # print angleLists
    # print timeLists


if __name__ == "__main__":
    #main()
    robotIP = "169.254.50.227"
    PORT = 9559
    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e

    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 10)
    
    try:
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)

    # turn function
    # @param degree should be in rad formulation
    step = 5
    moveForward(step)
    
