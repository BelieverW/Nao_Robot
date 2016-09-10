#IMPORT ALL THE NECESSARY LIBRARIES
import sys
#sys.path.append('/usr/local/lib/python2.7/site-packages/naoqi')
from naoqi import ALProxy
import motion
import Image
import almath
import math
import thread
import cv2 as cv
import numpy as np
from Detect_Ball2 import *

#SET THE ROBOT IP
robotIP="169.254.50.227"
R_min = 200
R_max = 250
G_min = 80
G_max = 120  
B_min = 20
B_max = 70
#The coefficients for distance fitting
a = 8.874*(10**6)
p = -1.891
#The coefficients for width fitting
p1 = 0.2365
p2 = -3.937
#Gear Para
width = 640.0
left_x = width/3
orig_x = width/2
right_x = width*2/3

#BEFORE STARTING WE INITIALISE THE 'MOTIONPROXY','POSTUREPROXY'
#AND 'TEXT TO SPEECH PROXY' OBJECTS
motionProxy  = ALProxy("ALMotion", robotIP, 9559)
postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
ttsProxy     = ALProxy("ALTextToSpeech", robotIP, 9559)

#isAbsolute variable is used wherever functions require a boolean argument
isAbsolute   = True

#The parameter of Robot
UA = UpperArmLength = 105
LA = LowerArmLength = 55.95+57.75

#WE USE CARTESIAN CONTROL ALSO
#We define the space and axis mask required for cartesian control of NAO
space      = motion.FRAME_ROBOT
axisMask   = almath.AXIS_MASK_ALL

#isAbsolute vari able is used wherever functions require a boolean argument
isAbsolute = False

def Detect_Simple(IP, PORT):
  camProxy = ALProxy("ALVideoDevice", IP, PORT)
  resolution = 2    # VGA
  colorSpace = 11   # RGB
  videoClient = camProxy.subscribeCamera("test", 1, resolution, colorSpace, 5)
  naoImage = camProxy.getImageRemote(videoClient)
  camProxy.unsubscribe(videoClient)
  imageWidth = naoImage[0]
  imageHeight = naoImage[1]
  array = naoImage[6]
  im = Image.fromstring("RGB", (imageWidth, imageHeight), array)
  im.save("camImage.png", "PNG")
  img0 = cv.imread('camImage.png')
  [b,g,r] = cv.split(img0)
  r = np.logical_and(r>=R_min,r<=R_max)
  g = np.logical_and(g>=G_min,g<=G_max)
  b = np.logical_and(b>=B_min,b<=B_max)
  img1 = np.logical_and(r,g)
  img1 = np.logical_and(b,img1)
  img1 = np.array(img1,dtype=np.uint8)
  contours, hierarchy = cv.findContours(img1,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)
  max = 0
  k = 0
  for i in range(0,len(contours)):
      tmp = cv.contourArea(contours[i])
      if tmp > max :
        max = tmp
        k = i
  if max > 0:
    M = cv.moments(contours[k])
    Cx = int( M['m10']/M['m00'] )
    Cy = int( M['m01']/M['m00'] )
  else:
    Cx = 0
    Cy = 0
  return Cx,Cy


#Righthand control
def righthand(px,y,z):
    pz = -z
    py = -y
    length = math.sqrt(px*px+py*py+pz*pz)
    tmp = (UA*UA+LA*LA-length*length)/(2*UA*LA)
    if tmp > 1:
        tmp = 1
    elif tmp < -1:
        tmp = -1
    thetha4 = -math.acos(tmp)+math.pi
    for i in range(-239,239,1):
        thetha1 = -i/2.0*math.pi/180
        tmp = (px*math.sin(thetha1)-py*math.cos(thetha1))/(LA*math.sin(thetha4))
        if tmp>1 or tmp<-1:
            continue
        thetha3 = math.asin(tmp)
        if thetha3 >math.pi*2/3 or thetha3 < -math.pi*2/3:
            continue
        B = pz
        A = px*math.cos(thetha1)+py*math.sin(thetha1)
        G1 = B*LA*math.cos(thetha4)+UA*B-A*LA*math.cos(thetha3)*math.sin(thetha4)
        G2 = A*LA*math.cos(thetha4)+UA*A+B*LA*math.cos(thetha3)*math.sin(thetha4)
        if G2!=0:
            thetha2 = math.atan2(G1,G2)
        else:
            thetha2 = (1 if (G1>0) else -1)*math.pi/2
        if thetha2 > 0 or thetha2 < -math.pi/2:
            continue
        break;
    if i == 239:
        return null
    angle1 = thetha1
    angle2 = thetha2
    angle3 = thetha3
    angle4 = thetha4

    angleLists = [[angle1],[angle2],[angle3],[angle4]]
    #print "angle1 =",angle1*180/math.pi
    #print "angle2 =",angle2*180/math.pi
    #print "angle3 =",angle3*180/math.pi
    #print "angle4 =",angle4*180/math.pi
    return angleLists

#Lefthand control
def lefthand(px,py,pz):
    length = math.sqrt(px*px+py*py+pz*pz)
    tmp = (UA*UA+LA*LA-length*length)/(2*UA*LA)
    if tmp > 1:
        tmp = 1
    elif tmp < -1:
        tmp = -1
    thetha4 = math.acos(tmp)-math.pi
    for i in range(-239,239,1):
        thetha1 = -i/2.0*math.pi/180
        tmp = (px*math.sin(thetha1)-py*math.cos(thetha1))/(LA*math.sin(thetha4))
        if tmp>1 or tmp<-1:
            continue
        thetha3 = math.asin(tmp)
        if thetha3 >math.pi*2/3 or thetha3 < -math.pi*2/3:
            continue
        B = pz
        A = px*math.cos(thetha1)+py*math.sin(thetha1)
        G1 = B*LA*math.cos(thetha4)+UA*B-A*LA*math.cos(thetha3)*math.sin(thetha4)
        G2 = A*LA*math.cos(thetha4)+UA*A+B*LA*math.cos(thetha3)*math.sin(thetha4)
        if G2!=0:
            thetha2 = math.atan2(G1,G2)
        else:
            thetha2 = (1 if (G1>0) else -1)*math.pi/2
        if thetha2 < 0 or thetha2 > math.pi/2:
            continue
        break;
    if i == 239:
        return null
    angle1 = thetha1
    angle2 = thetha2
    angle3 = thetha3
    angle4 = thetha4
    angleLists = [[angle1],[angle2],[angle3],[angle4]]
    #print "angle1 =",angle1*180/math.pi
    #print "angle2 =",angle2*180/math.pi
    #print "angle3 =",angle3*180/math.pi
    #print "angle4 =",angle4*180/math.pi
    return angleLists

def caltimes(t,t1,t2):
    i = 0
    times = [[i*t for i in range(1,t1+1)] for j in range(0,4)]+[[i*t for i in range(1,t2+1)] for j in range(0,4)]
    return times

def xdot(ov,nv,n):
    for i in range(0,n):
        ov[i] = ov[i]+nv[i]
    return ov

def Lline(px,py,pz):
    n = 1
    names      = ["LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll"]
    isAbsolute = True
    FinalLists = lefthand(px,py,pz)
    times = caltimes(2,n*2*2*n+1,n*2*n*2+1)
    for j in range(0,n):
        for i in range(0,n*2):
            angleLists = lefthand(px,py-i*py/n,pz)
            FinalLists = xdot(FinalLists,angleLists,4)
        for i in range(0,n*2):
            angleLists = lefthand(px,i*py/n,pz)
            FinalLists = xdot(FinalLists,angleLists,4)
    motionProxy.angleInterpolation(names, FinalLists, times, isAbsolute)

def LcubicRcircle(px,py,pz):
    n = 5
    names      = ["LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll"]
    isAbsolute = True
    FinalLists = lefthand(px,py,pz)+righthand(px,py,pz/2)
    times = caltimes(0.5,1+8*n,1+8*n)
    db = math.pi/10
    b = 0
    for i in range(0,n):
        angleLists = lefthand(px,py,pz)+righthand(px,py*math.cos(b),py+py*math.sin(b))
        FinalLists = xdot(FinalLists,angleLists,8)
        b += db
        angleLists = lefthand(px,0,pz)+righthand(px,py*math.cos(b),py+py*math.sin(b))
        FinalLists = xdot(FinalLists,angleLists,8)
        b += db
        angleLists = lefthand(px,-py,pz)+righthand(px,py*math.cos(b),py+py*math.sin(b))
        FinalLists = xdot(FinalLists,angleLists,8)
        b += db
        angleLists = lefthand(px,-py,pz/2)+righthand(px,py*math.cos(b),py+py*math.sin(b))
        FinalLists = xdot(FinalLists,angleLists,8)
        b += db
        angleLists = lefthand(px,-py,0)+righthand(px,py*math.cos(b),py+py*math.sin(b))
        FinalLists = xdot(FinalLists,angleLists,8)
        b += db
        angleLists = lefthand(px,0,0)+righthand(px,py*math.cos(b),py+py*math.sin(b))
        FinalLists = xdot(FinalLists,angleLists,8)
        b += db
        angleLists = lefthand(px,py,0)+righthand(px,py*math.cos(b),py+py*math.sin(b))
        FinalLists = xdot(FinalLists,angleLists,8)
        b += db
        angleLists = lefthand(px,py,pz/2)+righthand(px,py*math.cos(b),py+py*math.sin(b))
        FinalLists = xdot(FinalLists,angleLists,8)
        b += db
    motionProxy.angleInterpolation(names, FinalLists, times, isAbsolute)

def LR(px,py,pz,unit,time):
    names      = ["LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll"]
    isAbsolute = True
    FinalLists = lefthand(px,py+unit,pz)+righthand(px,-py+unit,pz)
    times = caltimes(time,1,1)
    motionProxy.angleInterpolation(names, FinalLists, times, isAbsolute)

def TaskInit():
    px = (LA+UA)*math.cos(math.pi/4)*math.cos(math.pi/6)*5/4
    py = (LA+UA)*math.cos(math.pi/4)*math.sin(math.pi/6)*1/2
    pz = 0
    LR(px,py,pz,0,2)

def armmove(gear):
    speed = 0.5
    px = (LA+UA)*math.cos(math.pi/4)*math.cos(math.pi/6)*5/4
    py = (LA+UA)*math.cos(math.pi/4)*math.sin(math.pi/6)*1/2
    pz = 0
    unit = 40
    LR(px,py,pz,gear*unit,speed)

def ContinuousGear(pos,speed):
    px = (LA+UA)*math.cos(math.pi/4)*math.cos(math.pi/6)*5/4
    py = (LA+UA)*math.cos(math.pi/4)*math.sin(math.pi/6)*1/2
    pz = 0
    LR(px,py,pz,gear*pos,speed)

def DropBall(motionProxy,LorR):
    px = (LA+UA)*math.cos(math.pi/4)*math.cos(math.pi/6)*5/4
    py = (LA+UA)*math.cos(math.pi/4)*math.sin(math.pi/6)*1/2
    pz = 0
    isAbsolute = True
    times = caltimes(2,1,1)
    if LorR==1:
      names      = ["RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RHand"]
      FinalLists = righthand(px,py,pz)+[[1]]
    else:
      names      = ["LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","LHand"]
      FinalLists = lefthand(px,-py,pz)+[[1]]
    motionProxy.angleInterpolation(names, FinalLists, times, isAbsolute)
    if LorR==1:
      names      = ["RHand"]
    else:
      names      = ["LHand"]
    FinalLists = [[0]]
    times = caltimes(1,1,1)
    motionProxy.angleInterpolation(names, FinalLists, times, isAbsolute)
    
    px = (LA+UA)*math.cos(math.pi/4)*math.cos(math.pi/6)*5/4
    py = (LA+UA)*math.cos(math.pi/4)*math.sin(math.pi/6)*1/2
    pz = 0
    if LorR==1:
      names      = ["RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RWristYaw"]
      FinalLists = righthand(-px,py,pz+4)+[[math.radians(-80)]]
    else:
      names      = ["LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","LWristYaw"]
      FinalLists = lefthand(-px,-py,pz+4)+[[math.radians(80)]]
    times = caltimes(4,1,1)
    motionProxy.angleInterpolation(names, FinalLists, times, isAbsolute)

    px = -(LA+UA)*math.sin(math.pi/6)
    py = (LA+UA)*math.cos(math.pi/6)
    pz = 0
    if LorR==1:
      names      = ["RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RWristYaw","RHand"]
      FinalLists = righthand(-px,py,pz)+[[math.radians(0)],[1]]
    else:
      names      = ["LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","LWristYaw","LHand"]
      FinalLists = lefthand(-px,-py,pz)+[[math.radians(0)],[1]]
    #times = caltimes(1,1,1)

    px = -(LA+UA)*math.sin(math.pi/3)
    py = -(LA+UA)*math.cos(math.pi/3)
    pz = 0
    if LorR==1:
      names      = ["RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RWristYaw","RHand"]
      FinalLists = xdot(FinalLists,righthand(-px,py,pz)+[[math.radians(0)],[1]],6)
    else:
      names      = ["LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","LWristYaw","LHand"]
      FinalLists = xdot(FinalLists,lefthand(-px,-py,pz)+[[math.radians(0)],[1]],6)

    times = caltimes(0.3,2,2)
    #motionProxy.angleInterpolation(names, FinalLists, times, isAbsolute)
    
    px = -(LA+UA)*math.sin(math.pi/6)
    py = -(LA+UA)*math.cos(math.pi/6)
    pz = 0
    if LorR==1:
      names      = ["RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RWristYaw","RHand"]
      FinalLists = xdot(FinalLists,righthand(-px,py,pz)+[[math.radians(0)],[1]],6)
    else:
      names      = ["LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","LWristYaw","LHand"]
      FinalLists = xdot(FinalLists,lefthand(-px,-py,pz)+[[math.radians(0)],[1]],6)
    
    times = xdot(times,caltimes(1,1,1),6)
    '''def caltimes(t,t1,t2):
    i = 0
    times = [[i*t for i in range(1,t1+1)] for j in range(0,4)]+[[i*t for i in range(1,t2+1)] for j in range(0,4)]'''
    motionProxy.angleInterpolation(names, FinalLists, times, isAbsolute)
  
#THIS IS THE DEFINITION OF THE MAIN FUNCTION
def main(robotIP): 
    #ALL THE DANCE MOVE FUNCTIONS ARE CALLED ONE BY ONE
    postureProxy.goToPosture("StandInit", 0.2)
    #--------------------------------------------------------                 
    #px = UA
    #py = LA
    #pz = LA
    LorR = -1
    '''isAbsolute = True
    times = caltimes(1,1,1)
    px = UA
    py = -LA*math.sin(math.pi/4)
    pz = LA*math.sin(math.pi/4)
    FinalLists = righthand(px,py,pz)
    motionProxy.angleInterpolation(names, FinalLists, times, isAbsolute)'''
    #--------------------------------------------------------
    #SET NAO TO INITIAL POSE
    #TaskInit()
    #KeepBalance(robotIP,9559)
    '''re=1
    while re==1:
      res,LorR,re = FindBall(robotIP, 9559, motionProxy)
      if re==1:
        time.sleep(3)
    print res
    px = res[0]#(LA+UA)*math.cos(math.pi/4)*math.cos(math.pi/6)*5/4
    py = res[1]#(LA+UA)*math.cos(math.pi/4)*math.sin(math.pi/6)*1/2
    pz = res[2]
    isAbsolute = True
    times = caltimes(2,1,1)
    print LorR
    if LorR==1:
      names      = ["RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RHand"]
      FinalLists = righthand(px,py,pz)+[[1]]
    else:
      names      = ["LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","LHand"]
      FinalLists = lefthand(px,-py,pz)+[[1]]
    motionProxy.angleInterpolation(names, FinalLists, times, isAbsolute)
    if LorR==1:
      names      = ["RHand"]
    else:
      names      = ["LHand"]
    FinalLists = [[0]]
    times = caltimes(1,1,1)
    time.sleep(3)
    motionProxy.angleInterpolation(names, FinalLists, times, isAbsolute)'''
    DropBall(motionProxy,LorR)
    

#CALL TO THE MAIN FUNCTION WHICH BEGINS THE DANCE
#BY CALLING ALL THE OTHER FUNCTIONS    
main(robotIP)
