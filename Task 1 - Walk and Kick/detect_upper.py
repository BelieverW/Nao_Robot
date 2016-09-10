import sys
import Image
from naoqi import ALProxy
import cv2 as cv
import numpy as np
#from matplotlib import pyplot as plt

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

def Detect_Ball_Upper(IP, PORT):

  camProxy = ALProxy("ALVideoDevice", IP, PORT)
  resolution = 2    # VGA
  colorSpace = 11   # RGB

  videoClient = camProxy.subscribeCamera("test", 0, resolution, colorSpace, 5)
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
    Y = a*(Cy**p)
    X = p1*abs(Cx-320)+p2
    if Cx < 320:
      X = -X
  else:
    X = 0
    Y = 0
  return X,Y



if __name__ == '__main__':
  IP = "169.254.75.194"  # Replace here with your NaoQi's IP address.
  PORT = 9559
  X,Y = Detect_Ball_Upper(IP,PORT)
  print X,Y