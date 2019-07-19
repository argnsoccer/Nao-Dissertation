# -*- encoding: UTF-8 -*-
# Get an image from NAO. Display it and save it using PIL.
# adapted by Andre Gras for red circle detection in image... will adapt further for real-time

import sys
import time
import cv2
import numpy as np
import math

# Python Image Library
from PIL import Image

from naoqi import ALProxy

IP = "192.168.137.238"  # Replace here with your NaoQi's IP address.
PORT = 9559

  
#motion = ALProxy("ALMotion", IP, PORT)
#tts = ALProxy("ALTextToSpeech", IP, PORT)

RED_MIN_LOWER = np.array([0, 150, 100], np.uint8)
RED_MAX_LOWER = np.array([10, 255, 255], np.uint8)

RED_MIN_UPPER = np.array([160, 150, 100], np.uint8)
RED_MAX_UPPER = np.array([179, 255, 255], np.uint8)

def takeImage():
  """
  Get an image from Nao
  """

  camProxy = ALProxy("ALVideoDevice", IP, PORT)
  resolution = 2    # VGA
  colorSpace = 11   # RGB

  videoClient = camProxy.subscribe("python_client", resolution, colorSpace, 5)

  # Get a camera image.
  # image[6] contains the image data passed as an array of ASCII chars.
  naoImage = camProxy.getImageRemote(videoClient)

  camProxy.unsubscribe(videoClient)

  # Now we work with the image returned and save it as a PNG  using ImageDraw
  # package.

  # Get the image size and pixel array.
  imageWidth = naoImage[0]
  print imageWidth
  imageHeight = naoImage[1]
  print imageHeight
  array = naoImage[6]

  # Create a PIL Image from our pixel array.
  im = Image.frombytes("RGB", (imageWidth, imageHeight), array)

  # Save the image.
  im.save("camImage.png", "PNG")
  process()

def process():
  """
  turn the image into hsv and make red mask to do CHT
  """

  image = cv2.imread("camImage.png")
  #blur to improve circle detection through noise
  #get HSV mapping for red values
  hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

  #create mask for lower red HSV threshold
  mask_lower = cv2.inRange(hsv, RED_MIN_LOWER, RED_MAX_LOWER)


  #create mask for upper red HSV threshold
  mask_upper = cv2.inRange(hsv, RED_MIN_UPPER, RED_MAX_UPPER)


  #combine both masks
  mask = cv2.addWeighted(mask_lower, 1.0, mask_upper, 1.0, 0.0)

  kernel = np.ones((5,5),np.uint8)

  #erode and dilate then dilate and erode    
  mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
  mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

  cv2.imwrite("hsvMask.png", mask)

    
  #apply Gaussian Blur to improve detection and remove some noise
  blur = cv2.GaussianBlur(mask, (11,11), 0)
  cv2.imwrite("blurredImg.png", blur)

  #perform CHT
  circles = cv2.HoughCircles(blur, cv2.cv.CV_HOUGH_GRADIENT, 1, 40, 800, 150, 20, 0)

  try:
    circles = np.uint16(np.around(circles))
  except AttributeError:
    print("No Circles Found! Adjust parameters of CHT.")
      
            
  try:
     for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(image,(i[0],i[1]),2,(0,0,255),3)
  except TypeError:
     print("No Circles Found! Adjust parameters of CHT")
    
  #write out image with drawn circles
  cv2.imwrite("detectedCircles.png", image)
  

if __name__ == '__main__':
  # Read IP address from first argument if any.
  if len(sys.argv) > 1:
    IP = sys.argv[1]

  
    
  naoImage = takeImage()
  
