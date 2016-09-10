# -*- encoding: UTF-8 -*-
# Get an image from NAO. Display it and save it using PIL.

import sys
import time
import cv2
import numpy as np
import math
from cv2.cv import *

# Python Image Library
import Image

from naoqi import ALProxy

angle = 11.75
catchname="catch.png"
detectname="detect.jpg"
inm = np.array([[0.000895884710808678, 0, 0],[0,0.000891628101607045,0],[-0.580306123792026,-0.450885446722372,1]])
#rot = np.array([[1,0,0],[0,0.965925826289068,-0.258819045102521],[0,0.258819045102521,0.965925826289068]])
rot = np.array([[1,0,0],[0,math.cos(angle/180*math.pi),-math.sin(angle/180*math.pi)],[0,math.sin(angle/180*math.pi),math.cos(angle/180*math.pi)]])
#detected paras
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

def CatchImage(IP, PORT):
  """
  First get an image from Nao, then show it on the screen with PIL.
  """

  camProxy = ALProxy("ALVideoDevice", IP, PORT)
  resolution = 3    # VGA
  colorSpace = 11   # RGB

  videoClient = camProxy.subscribeCamera("python_client", 0, resolution, colorSpace, 5)

  t0 = time.time()

  # Get a camera image.
  # image[6] contains the image data passed as an array of ASCII chars.
  naoImage = camProxy.getImageRemote(videoClient)

  t1 = time.time()

  # Time the image transfer.
  print "acquisition delay ", t1 - t0

  camProxy.unsubscribe(videoClient)


  # Now we work with the image returned and save it as a PNG  using ImageDraw
  # package.

  # Get the image size and pixel array.
  imageWidth = naoImage[0]
  imageHeight = naoImage[1]
  array = naoImage[6]

  # Create a PIL Image from our pixel array.
  im = Image.fromstring("RGB", (imageWidth, imageHeight), array)

  # Save the image.
  im.save(catchname, "PNG")
  #im.show()

def DetectBall():
  img0 = cv2.imread(catchname)
  [b,g,r] = cv2.split(img0)
  r = np.logical_and(r>=R_min,r<=R_max)
  g = np.logical_and(g>=G_min,g<=G_max)
  b = np.logical_and(b>=B_min,b<=B_max)
  img1 = np.logical_and(r,g)
  img1 = np.logical_and(b,img1)
  img1 = np.array(img1,dtype=np.uint8)
  contours, hierarchy = cv2.findContours(img1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
  max = 0
  k = 0
  for i in range(0,len(contours)):
      tmp = cv2.contourArea(contours[i])
      if tmp > max :
        max = tmp
        k = i
  if max > 0:
    M = cv2.moments(contours[k])
    Cx = int( M['m10']/M['m00'] )
    Cy = int( M['m01']/M['m00'] )
  else:
    Cx = 0
    Cy = 0
  return [Cx,Cy]

def CalDistance(xy):
  x = xy[0]
  y = xy[1]
  xyz = np.array([x,y,1])
  res1 = np.dot(xyz,inm)
  res2 = np.dot(res1,rot)
  res3 = 1.0*res2/res2[1]*500
  return [res3[0],res3[2]]

def CalDistance2(xy, da):
  bangle = angle+da
  print bangle
  rot2 = np.array([[1,0,0],[0,math.cos(bangle/180*math.pi),-math.sin(bangle/180*math.pi)],[0,math.sin(bangle/180*math.pi),math.cos(bangle/180*math.pi)]])
  x = xy[0]
  y = xy[1]
  xyz = np.array([x,y,1])
  res1 = np.dot(xyz,inm)
  res2 = np.dot(res1,rot2)
  res3 = res2/res2[1]*500
  return [res3[0],res3[2]]

def CalDistance3(xy):
  x = xy[0]
  y = xy[1]
  xyz = np.array([x,y,1])
  res1 = np.dot(xyz,inm)
  res2 = np.dot(res1,rot)
  res3 = res2/res2[1]
  return res3

def DoubleJudgement(res1,res2,r):
  x1 = res1[0]
  x2 = res2[0]
  z1 = res1[2]
  z2 = res2[2]
  h = -2*r*(z2-z1)/(x1*x1+z1*z1-x2*x2-z2*z2)
  if res1[2]*h<0:
    h=-h
  return [res1[0]*h,res1[1]*h,res1[2]*h]

def FindBall(IP, PORT, motionProxy):
  CatchImage(IP, PORT)
  coord = DetectBall()
  res1 = CalDistance3(coord)
  re = 0
  print res1
  print coord
  if coord[0]>640:
    theta = -10
  else:
    theta = 10
  theta = math.radians(theta)
  motionProxy.setAngles("HeadYaw", theta, 0.02)
  if coord[0]==0:
    re = 1
  time.sleep(3)
  CatchImage(IP, PORT)
  coord = DetectBall()
  res2 = CalDistance3(coord)
  print res2
  print coord
  motionProxy.setAngles("HeadYaw", 0, 0.02)
  r = 50
  res = DoubleJudgement(res1,res2,r)
  print res
  if res1[0]<0:
    res[0] = -res[0]-98
    LorR = -1
    xyz = [res[1]+140,120-res[2],res[0]]
  else:
    res[0] = res[0]-98
    LorR = 1
    xyz = [res[1]+120,160-res[2],res[0]]
  if coord[0]==0:
    re = 1
  return xyz,LorR,re

if __name__ == '__main__':
  IP = "169.254.50.227"  # Replace here with your NAOqi's IP address.
  PORT = 9559
  try:
    postureProxy = ALProxy("ALRobotPosture", IP, PORT)
  except Exception, e:
    print "Could not create proxy to ALRobotPosture"
    print "Error was: ", e
  try:
    motionProxy = ALProxy("ALMotion", IP, PORT)
  except Exception,e:
    print "Could not create proxy to ALMotion"
    print "Error was: ",e
    sys.exit(1)
    
  postureProxy.goToPosture("StandInit", 0.2)

  # Read IP address from first argument if any.
  if len(sys.argv) > 1:
    IP = sys.argv[1]

  FindBall(IP, PORT, motionProxy)
  '''for i in range(0,20):
    res = DoubleJudgement(res1,res2,r+i*1)
    print i,",",res'''
  #print rot



