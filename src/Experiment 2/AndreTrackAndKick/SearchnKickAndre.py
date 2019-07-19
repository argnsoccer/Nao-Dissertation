import qi
import argparse
import time
import math
from functools import partial
import numpy as np
import motion as mot

# Python Image Library
from PIL import Image
# OpenCV Libraries
import cv2
import cv2.cv as cv

''' ROBOT CONFIGURATION '''
robotIP = "192.168.137.238"
ses = qi.Session()
ses.connect(robotIP)
per = qi.PeriodicTask()
motion = ses.service('ALMotion')
posture = ses.service('ALRobotPosture')
tracker = ses.service('ALTracker')
video = ses.service('ALVideoDevice')
tts = ses.service('ALTextToSpeech')
landmark = ses.service('ALLandMarkDetection')
memory = ses.service('ALMemory')

resolution = 2    # VGA
colorSpace = 11   # RGB
trial_number = 12
path = ''

def take_pics(angleScan, CameraIndex):
    """ 
    Takes pictures in a scan of environment (Written by David Lavy and Travis Marshall)
    """

    names = "HeadYaw"
    useSensors = False
    motionAngles = []
    maxAngleScan = angleScan

    motion.angleInterpolationWithSpeed("Head", [-maxAngleScan, 0.035], 0.1)
    pic(path + 'bFound0.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [0, 0.035], 0.1)
    pic(path + 'bFound1.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [maxAngleScan, 0.035], 0.1)
    pic(path + 'bFound2.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    centers = analyze_img2()
    return [centers, motionAngles]
 
def locate_ball(centers, rot_angles):
    """ 
    Find the ball and center its look to it, otherwise back to 0 and rotate again (Written by David Lavy and Travis Marshall)
    """
    index = numBalls(centers)
    if len(index) == 0:
        string = "I don't see the ball."
        ang = 100
        state = 0
        RF = 0
    elif len(index) == 1:
        a = index[0]
        string = "I need to get a better look at the ball."
        ang = rot_angles[a][0]
        # ang = ang.item()
        state = 1
        RF = 0
        motion.angleInterpolationWithSpeed("Head", [ang, 0.035], 0.1)
    else:
        string = "I see the ball."
        a = index[0]
        print a
        b = index[1]
        print b
        RF = (rot_angles[b][0] - rot_angles[a][0]) / (centers[a][1] - centers[b][1])
        print RF
        ang = rot_angles[a][0] - (320 - centers[a][1])*RF
        ang = ang/100
        ang = ang.item()
        state = 2
        motion.angleInterpolationWithSpeed("Head", [ang, 0.035], 0.1)
    print ang
    tts.say(string)
    return [ang, state, RF]

# 
def move_head(angleScan):
    """ 
    Move HeadYaw from [-angleScan;angleScan] (Written by David Lavy and Travis Marshall)
    """
    print 'moving head'
    angleLists = [[0, angleScan]]
    timeLists = [[1.0, 2.0]]
    motion.angleInterpolation("HeadYaw", angleLists, timeLists, True)


def CenterOfMassUp(image):
    """ 
    Calculate position and radius of circle using the Circular Hough Transform
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lowera = np.array([157, 120, 100])
    uppera = np.array([180, 255, 255])
    lowerb = np.array([0, 120, 100])
    upperb = np.array([12, 250, 255])
    x = 0
    y = 0
    mask1 = cv2.inRange(hsv, lowera, uppera)
    mask2 = cv2.inRange(hsv, lowerb, upperb)
    mask = cv2.add(mask1,mask2)
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # cv2.imwrite("hsvMask.png", mask)
    
    blur = cv2.GaussianBlur(mask, (9,9), 0)
    # cv2.imwrite("blurredImg.png", blur)

    #perform CHT
    circles = cv2.HoughCircles(blur, cv2.cv.CV_HOUGH_GRADIENT, 1, 40, 800, 100, 20, 0)

    try:
      circles = np.uint16(np.around(circles))
    except AttributeError:
      print("No Circles Found! Adjust parameters of CHT.")

     
    try:
      for i in circles[0,:]:
        if i[0] > 0:
            x = i[0]
        else:
            x = 0
        if i[1] > 0:
            y = i[1]
        else:
            y = 0
            
    except TypeError:
      print("No Circles Found! Adjust parameters of CHT")

    CM = [x, y]

    return CM

def CenterOfMassDown(image):
    """ 
    Calculate position and radius of circle using the Circular Hough Transform
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lowera = np.array([160, 150, 100])
    uppera = np.array([180, 255, 255])
    lowerb = np.array([0, 150, 100])
    upperb = np.array([10, 250, 255])
    x = 0
    y = 0
    mask1 = cv2.inRange(hsv, lowera, uppera)
    mask2 = cv2.inRange(hsv, lowerb, upperb)
    mask = cv2.add(mask1,mask2)
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # cv2.imwrite("hsvMask.png", mask)
    
    blur = cv2.GaussianBlur(mask, (9,9), 0)
    # cv2.imwrite("blurredImg.png", blur)

    #perform CHT
    circles = cv2.HoughCircles(blur, cv2.cv.CV_HOUGH_GRADIENT, 2, 40, 400, 100, 15, 0)

    try:
      circles = np.uint16(np.around(circles))
    except AttributeError:
      print("No Circles Found! Adjust parameters of CHT.")

     
    try:
      for i in circles[0,:]:
        if i[0] > 0:
            x = i[0]
        else:
            x = 0
        if i[1] > 0:
            y = i[1]
        else:
            y = 0
    except TypeError:
      print("No Circles Found! Adjust parameters of CHT")

    CM = [x, y]

    return CM

# 
def analyze_img():
    """ 
    Find the center position of ball (written by David Lavy and Travis Marshall)
    """
    CM = []
    for i in range(0, 7):
        img = cv2.imread(path + "camImage" + str(i) + ".png")
        cm = CenterOfMassUp(img)
        CM.append(cm)
    return CM

def analyze_img2():
    """ 
    Find the center position of ball (written by David Lavy and Travis Marshall)
    """
    CM = []
    for i in range(0, 3):
        img = cv2.imread(path + "bFound" + str(i) + ".png")
        cm = CenterOfMassUp(img)
        CM.append(cm)
    return CM


def scan_area(_angleSearch, CameraIndex):
    """ 
    Scan for ball in front of robot (written by David Lavy and Travis Marshall)
    """
    # Search angle where angle of rotation is [-maxAngleScan;+maxAngleScan]
    names = "HeadYaw"
    useSensors = False
    motionAngles = []
    maxAngleScan = _angleSearch

    motion.angleInterpolationWithSpeed("Head", [-maxAngleScan, 0.035], 0.1)
    pic(path + 'camImage0.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [-2*maxAngleScan/3, 0.035], 0.1)
    pic(path + 'camImage1.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [-maxAngleScan/3, 0.035], 0.1)
    pic(path + 'camImage2.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [0, 0.035], 0.1)
    pic(path + 'camImage3.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [maxAngleScan/3, 0.035], 0.1)
    pic(path + 'camImage4.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [2*maxAngleScan/3, 0.035], 0.1)
    pic(path + 'camImage5.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    motion.angleInterpolationWithSpeed("Head", [maxAngleScan, 0.035], 0.1)
    pic(path + 'camImage6.png', CameraIndex)
    commandAngles = motion.getAngles(names, useSensors)
    motionAngles.append(commandAngles)
    print str(commandAngles)
    centers = analyze_img()
    return [centers, motionAngles]

# Index of pictures that contains the ball
def numBalls(CM):
    """
    Takes in the CM list, outputs indices of frames containing balls (Written by David Lavy and Travis Marshall)
    """
    index = []
    for i in range(len(CM)):
        if CM[i] != [0, 0]:
            index.append(i)
    return index

# 
def rotate_center_head(centers, rot_angles):
    """
    Find the ball and center its look to it, otherwise back to 0 and rotate again (Written by David Lavy and Travis Marshall)
    """
    index = numBalls(centers)
    found = 1
    if len(index) == 0:
        string = "I don't see the ball."
        ang = 100
        found = 0
    elif len(index) == 1:
        a = index[0]
        string = "I think I see the ball."
        ang = rot_angles[a][0]
    else:
        string = "I see the ball."
        a = index[0]
        b = index[1]
        den = 3
        if len(index) < 3:
            ang = (rot_angles[b][0] + rot_angles[a][0])/2
        else:
            c = index[2]
            ang = (rot_angles[b][0] + rot_angles[a][0] + rot_angles[c][0])/3
    print ang
    motion.angleInterpolationWithSpeed("Head", [0, 0.035], 0.1)
    tts.say(string)
    return ang, found


def pic(_name, CameraIndex):
    """
    Takes a picture and saves it
    """	
    videoClient = video.subscribeCamera(
        "python_client", CameraIndex, resolution, colorSpace, 5)
    naoImage = video.getImageRemote(videoClient)
    video.unsubscribe(videoClient)
    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    im = Image.frombytes("RGB", (imageWidth, imageHeight), str(array))
    im.save(_name, "PNG")

# 
def initial_scan():
    """
    Function that will look for position of ball and return it (Written by David Lavy and Travis Marshall)
    """
    state = 0
    angleSearch = 60*math.pi/180
    ang = 100

    motion.moveInit()
    camIndex = 0  # Starts with upper camera

    state = 0
    while ang == 100:
        [CC, AA] = scan_area(angleSearch, camIndex)
        print CC
        ang, found = rotate_center_head(CC, AA)
        if found == 0 and camIndex == 1:
            state = state + 1
            camIndex = 0
            motion.moveTo(0, 0, 2*math.pi/3)
        elif found == 0 and camIndex == 0:
            camIndex = 1
        if state == 3:
            tts.say('I need to move to find the ball')
            motion.moveTo(0.3, 0, 0)
            state = 0

    else:
        motion.moveTo(0, 0, ang*7/6)
    pic(path + "ball_likely.png",0)
    [CC1, AA1] = take_pics(math.pi /9, camIndex)
    print CC1
    [ang, X, delta] = locate_ball(CC1, AA1)
    if ang == 100:
        camIndex = 2
    print 'Delta', delta
    print 'Ang', ang
    motion.moveTo(0, 0, ang*7/6)
    img=cv2.imread(path + "ball_likely.png")
    CM=CenterOfMassUp(img)
        
    return CM, delta, camIndex

def walkUp(cm, delta):
    """
    Walk towards the ball using the upper camera (Written by David Lavy and Travis Marshall)
    """
    idx = 1
    lowerFlag = 0
    print "Entering uppercam loop"
    motion.moveTo(0.2, 0, 0)
    while cm[0] < 420 and cm[0] > 0:
        pp = "ball_upfront"
        ext = ".png"
        im_num = path + pp+str(idx)+ext
        pic(im_num, 0)
        img = cv2.imread(im_num)
        cm = CenterOfMassUp(img)
        print cm
        if cm[0] == 0 and cm[1] == 0:
            # Scan the area with lower camera
            pic(path + 'lower.png', 1)
            img = cv2.imread(path + "lower.png")
            cm2 = CenterOfMassUp(img)
            lowerFlag = 1
            break
        else:
            alpha = (cm[1] - 320) * delta
            motion.moveTo(0.2, 0, alpha*7/6)
            idx = idx + 1
            continue
    if lowerFlag == 1:
        if cm2[0] == 0 and cm2[1] == 0:
            lostFlag = 1
            print 'I lost the ball'
        else:
            lostFlag = 0
            print 'I need to switch cameras'
    else:
        pic(path + 'lower.png', 1)
        img = cv2.imread(path + 'lower.png')
        cm2 = CenterOfMassUp(img)
        lostFlag = 0
    print "Exiting up loop"
    return lostFlag, cm2
    # motion.moveTo(0.15, 0, 0)

def walkDown(cm, delta):
    """
    Walk towards the ball using the lower camera (Written by David Lavy and Travis Marshall)
    """
    idx = 1
    pp = "ball_downfront"
    ext = ".png"
    print 'Entering lowercam loop'
    # motion.moveTo(0.2, 0, alpha*7/6)
    motion.moveTo(0.2, 0, 0)
    while cm[0] > 0 and cm[0] < 230:
        # motion.moveTo(0.2, 0, 0)
        im_num = path + pp+str(idx)+ext
        pic(im_num, 1)
        img = cv2.imread(im_num)
        cm = CenterOfMassUp(img)
        print im_num, cm
        if cm == [0, 0]:
            return 0, cm
        alpha = (cm[1] - 320) * delta
        motion.moveTo(0.2, 0, alpha*7/6)
        idx = idx + 1
    # Tilt the head so it can have a better look of the ball
    anglePitch = math.pi * 20.6 / 180
    motion.angleInterpolationWithSpeed("HeadPitch", anglePitch, 0.1)
    # print 'Pitching the head'
    # The threshold of 300 is equal to a distance of 15cm from the ball
    # The robot will do a small walk of 7cm and exit the loop
    print 'Entering sub-precise with cm[0]', cm[0]
    while cm[0] >= 0  and cm[0] < 300:
        im_num = path + pp+str(idx)+ext
        pic(im_num, 1)
        img = cv2.imread(im_num)
        cm = CenterOfMassDown(img)
        print im_num, cm
        if cm == [0, 0]:
            return 0, cm
        if cm[0] < 350:
            alpha = (cm[1] - 320) * delta
            motion.moveTo(0.07, 0, alpha*8/6)
        else: 
            break
        idx = idx + 1
    taskComplete = 1
    return taskComplete, cm

def getReady(cm, delta):
    # The ball should be at about 10cm roughly from the ball
    # Do the correction before it starts the loop
    idx = 1
    pp = "ball_precise"
    ext = ".png"
    im_num = path + pp+str(idx-1)+ext
    pic(im_num, 1)
    img = cv2.imread(im_num)
    cm = CenterOfMassDown(img)
    alpha = (cm[1] - 320) * delta
    motion.moveTo(0, 0, alpha*7/6)
    print 'Precising the position'
    print 'This is my cm[0]', cm[0]
    while cm[0] < 370:
        print 'Entering the loop'
        im_num = path + pp+str(idx)+ext
        pic(im_num, 1)
        img = cv2.imread(im_num)
        cm = CenterOfMassDown(img)
        print im_num, cm
        if cm == [0, 0]:
            return 0, cm
        if cm[0] < 405:
            alpha = (cm[1] - 320) * delta
            motion.moveTo(0.05, 0, alpha)
        else:
            break
        idx = idx + 1
    return 1
    # This should exit at a good distance to kick the ball

def kickBall():
    """
    Kicks the ball (Written by David Lavy and Travis Marshall)
    """

    # Activate Whole Body Balancer
    isEnabled  = True
    motion.wbEnable(isEnabled)

    # Legs are constrained fixed
    stateName  = "Fixed"
    supportLeg = "Legs"
    motion.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable   = True
    supportLeg = "Legs"
    motion.wbEnableBalanceConstraint(isEnable, supportLeg)

    # Com go to LLeg
    supportLeg = "LLeg"
    duration   = 1.0
    motion.wbGoToBalance(supportLeg, duration)

    # RLeg is free
    stateName  = "Free"
    supportLeg = "RLeg"
    motion.wbFootState(stateName, supportLeg)

    # RLeg is optimized
    effectorName = "RLeg"
    axisMask     = 63
    space        = mot.FRAME_TORSO


    # Motion of the RLeg
    dx      = 0.025                 # translation axis X (meters)
    dz      = 0.02                 # translation axis Z (meters)
    dwy     = 5.0*math.pi/180.0    # rotation axis Y (radian)


    times   = [1.0, 1.4, 2.1]
    isAbsolute = False

    targetList = [
      [-0.7*dx, 0.0, 1.1*dz, 0.0, +dwy, 0.0],
      [+2.2*dx, +dx, dz, 0.0, -dwy, 0.0],
      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

    motion.positionInterpolation(effectorName, space, targetList,
                                 axisMask, times, isAbsolute)


    # Example showing how to Enable Effector Control as an Optimization
    isActive     = False
    motion.wbEnableEffectorOptimization(effectorName, isActive)

    time.sleep(1.0)

    # Deactivate Head tracking
    isEnabled    = False
    motion.wbEnable(isEnabled)

    # send robot to Pose Init
    posture.goToPosture("StandInit", 0.5)

def set_head_position(_angle):
    fracSpeed = 0.2
    names = ['HeadYaw']
    motion.setAngles(names, _angle, fracSpeed)

def zero_head():
    motion.angleInterpolationWithSpeed("HeadYaw", 0, 0.1)

def main(robotIP, PORT=9559):

    #Wake up the robot
    motion.wakeUp()
    taskCompleteFlag = 0
    while taskCompleteFlag == 0:
        ballPosition, delta, camIndex = initial_scan()
        if camIndex == 0:
            zero_head()
            lost, CoM = walkUp(ballPosition, delta)
            if lost == 0:
                # Switch cameras
                time.sleep(0.2)
                video.stopCamera(0)
                video.startCamera(1)
                video.setActiveCamera(1)
                zero_head()
                # Walk to the ball using lower camera
                taskCompleteFlag, CoM1 = walkDown(CoM, delta)
                taskCompleteFlag = getReady(CoM1, delta)
            else:
                tts.say('I lost the ball, I need to rescan.')
                motion.moveTo(-0.2, 0, 0)
        elif camIndex == 1:
            # Switch cameras
            time.sleep(0.2)
            video.stopCamera(0)
            video.startCamera(1)
            video.setActiveCamera(1)
            zero_head()
            # Walk to the ball using lower camera
            taskCompleteFlag, CoM1 = walkDown(ballPosition, delta)
            taskCompleteFlag = getReady(CoM1, delta)
    kickBall()
    motion.rest()


if __name__ == "__main__":

    main(robotIP)
