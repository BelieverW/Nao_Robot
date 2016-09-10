import sys
import time
import almath
import math
from naoqi import ALProxy
from TurnLeft import * 
from TurnRight import *
from MoveForward import *
#from vision import *


# def Camera_Ball( robotIP, PORT ):
#     X,Y = Detect_Ball_Upper(robotIP, PORT)
#     if Y<90 or Y>180:
#         X,Y = Detect_Ball_Down(robotIP, PORT)
#     return X,Y

# def Search( robotIP, PORT ):
#     #middle view
#     X,Y = Detect_Key( robotIP, PORT )
#     if X!=0 or Y!=0:
#         print X,Y
#         tts.say("Target is found")
#         if X > 0:
#             return X-1.5,Y+7
#         else:
#             return X+1.5,Y+7
#     motionProxy = ALProxy("ALMotion", robotIP, PORT)
#     motionProxy.setStiffnesses("Body",1.0)
#     fractionMaxSpeed = 0.2
#     #right view
#     theta = math.radians(-30)
#     motionProxy.setAngles("HeadYaw", theta, fractionMaxSpeed)
#     time.sleep(1)
#     X,Y = Detect_Key( robotIP, PORT )
#     if X!=0 or Y!=0:
#         X = X-1.5
#         Y = Y+7
#         print X,Y
#         tts.say("Target is found")
#         XX = X*math.cos(-theta)+Y*math.sin(-theta)
#         YY = -X*math.sin(-theta)+Y*math.cos(-theta)
#         return XX+1.5,YY
#     #left view
#     motionProxy.setAngles("HeadYaw", -theta, fractionMaxSpeed)
#     time.sleep(1)
#     X,Y = Detect_Key( robotIP, PORT )
#     X = X-1.5
#     Y = Y+7
#     print X,Y
#     tts.say("Target is found")
#     XX = X*math.cos(theta)+Y*math.sin(theta)
#     YY = -X*math.sin(theta)+Y*math.cos(theta)
#     return XX-1.5,YY+2    

def main( PORT ):
    
    for i in range(3):
        # ATTENTION!!!!!!!!!
        # need to be changed
        # use vision.py to get the location of the target
        X,Y = Camera_Ball(robotIP, PORT)
        print X,Y

        if X==0 or Y==0:
            # can not find the target
            tts.say("I can not find the target")
        else:
            tts.say("I find the target")
            time.sleep(1)

            # calculate the angle
            # ATTENTION!!!!!!!!!
            # need to be changed
            theta = math.atan(X / Y)

            if abs(theta) > (5.0 * math.pi / 180):
                # rotate the robot
                if theta > 0:
                    tts.say("Turn Right")
                    turnRight(theta)
                else:
                    theta = -theta
                    tts.say("Turn Left")
                    turnLeft(theta)

            tts.say("Let's Move")

            # ATTENTION!!!!!!!!!
            # need to be changed
            # reuse vision.py to get the new location of the target
            X,Y = Camera_Ball(robotIP, PORT)

            if i != 2:
                if i == 0:
                    # assume that the length must be larger than 600mm
                    length = Y - 600
                    steps = int((length * 0.8 - 45) // 90)
                if i == 1:
                    length = Y - 600
                    steps = int ((length - 45) // 90)
                moveForward(steps)
            else:
                steps = int ((Y - 45) // 90)
                moveForwardAndKick(steps)
                tts.say("Oh Yes")

        postureProxy.goToPosture("StandInit", 10)
 

if __name__ == '__main__':
    
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

    # set audio     
    tts = ALProxy("ALTextToSpeech", robotIP, PORT)
    tts.setLanguage('English')

    # main function
    main( PORT )