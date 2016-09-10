# TurnLeft.py

# Nao Robot Turn Left
import sys
import time
from naoqi import ALProxy
from math import pi,sin,cos,asin,acos,atan2,sqrt

# The length of Thigh
ThighLength = 100.0
# The length of Tibia
TibiaLength = 102.90

HipHeight = -200
HipOffsetY = 50
AnkleHeight = 50
StepLength = 100

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
    z = z + 30
    angle[0] = atan2(y, -z)
    k = z * cos(angle[0]) - y * sin(angle[0])
    tmp = x * x + y * y + z * z - ThighLength * ThighLength - TibiaLength * TibiaLength
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


def turnRight(dre):
    names = ["RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll","LHipYawPitch"]
    motionProxy.setStiffnesses(names, 1.0)
    dre   = dre * 180.0 / pi
    up    = 25 - ThighLength - TibiaLength
    num   = 9


    # ?????? 60? or 50?
    move  = 65
    speed = 1
    k     = int(dre / 10)
    if k != 1.0 * dre / 10:
        k = k + 1
    hipYawPitch = 0.008 * -1 * dre / k

    angleLists  = [range(num) for i in range(len(names))]
    #angleLists2 = [range(1) for i in range(len(names))]

    flag = -1

    # move center of gravity to the center of right leg
    i = 0
    angLeft  = getAngles(0, flag * move, HipHeight)
    angRight = getAngles(0, flag * move, HipHeight)
    angleLists[10][i] = 0
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    # lift the left leg
    i = 1
    angLeft  = getAngles(0, flag * move, HipHeight)
    angRight = getAngles(0, flag * move, up)
    angleLists[10][i] = 0
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    # rotate the hipYawPitch
    i = 2
    angLeft  = getAngles(0, flag * move, HipHeight)
    angRight = getAngles(0, flag * move, up)
    angleLists[10][i] = hipYawPitch
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]
    i = 3
    angLeft  = getAngles(0, flag * move, HipHeight)
    angRight = getAngles(0, flag * move, HipHeight)
    angleLists[10][i] = hipYawPitch
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]
    # put down the left leg
    i = 4
    angLeft  = getAngles(0, 0, HipHeight)
    angRight = getAngles(0, 0, HipHeight)
    angleLists[10][i] = hipYawPitch
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    # change the center of gravity to the center of left leg
    i = 5
    angLeft  = getAngles(0, -flag * move, HipHeight)
    angRight = getAngles(0, -flag * move, HipHeight)
    angleLists[10][i] = hipYawPitch
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    # lift the right leg
    i = 6
    angLeft  = getAngles(0, -flag * move, up)
    angRight = getAngles(0, -flag * move, HipHeight)
    angleLists[10][i] = hipYawPitch
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    # rotate back the hipYawPitch and put down the right leg
    i = 7
    angLeft  = getAngles(0, -flag * move, HipHeight)
    angRight = getAngles(0, -flag * move, HipHeight)
    angleLists[10][i] = -0
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    #i=7
    i = 8
    angRight = getAngles(0, -0, HipHeight)
    angLeft  = getAngles(0, -0, HipHeight)
    angleLists[10][i] = -0
    for j in range(5):
        angleLists[j][i] = angRight[j]
    for j in range(5,10):
        angleLists[j][i] = angLeft[j - 5]

    time = [1, 2, 3, 4, 6, 7, 8, 9, 10]
    timeLists=[time for j in range(len(names))]

    for i in range(k):
        motionProxy.angleInterpolation(names, angleLists, timeLists, True)


    # finish the rotation
    angleLists2 = [range(1) for i in range(len(names))]
    angRight = getAngles(0, 0, HipHeight)
    angLeft  = getAngles(0, 0, HipHeight)
    i = 0
    angleLists2[10][i] = -0
    for j in range(5):
        angleLists2[j][i] = angRight[j]
    for j in range(5,10):
        angleLists2[j][i] = angLeft[j - 5]
    timeLists = speed
    motionProxy.angleInterpolation(names, angleLists2, timeLists, True)
    print "finished"



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
    postureProxy.goToPosture("StandInit", 8)

    try:
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)

    # turn function
    # @param degree should be in rad formulation
    turn(  2 * pi )
