import gym
from naoqi import ALProxy
from gym import error, spaces, utils
from gym.utils import seeding
import math
import numpy as np

from PIL import Image

from naoqi import ALProxy

IP = "127.0.0.1"  # Replace here with your NaoQi's IP address.
PORT = 9559
motionProxy = ALProxy("ALMotion", IP, PORT)
camProxy = ALProxy("ALVideoDevice", IP, PORT)
resolution = 7    # QQQVGA, 80x60, can make larger if it trains quickly enough
colorSpace = 11   # RGB


#resolution of camera and the state that updates every step
STATE_HEIGHT = 80
STATE_WIDTH = 60



# This is the one action the robot can take (with two changing parameters)

def rescale_actions(output, low, high):
  range = high - low
  temp = (output-low)/range
  return temp*range + low


class BasicEnv(gym.Env):
  metadata = {'render.modes': ['human', 'rgb_array', 'state_pixels']}

  def __init__(self):
    self.reward = 0.0
    self.prev_reward = 0.0
    self.x = 0.0
    self.ang = 0.0
    self.prevRed = 0.0
    self.red = 0.0
    self.prevBlue = 0.0
    self.blue = 0.0
    self.prevGreen = 0.0
    self.green = 0.0
    
    # 0 = Orient, 1 = Move Forward, Also need continuous degree to orient or amount to move
    self.action_space = spaces.Box(low = np.array([-math.pi/6, 0.0]), high = np.array([+math.pi/6, +0.3]), dtype = np.float64)
    
    #pixels coming in, RGB, need to scale down resolution to state pixels
    self.observation_space = spaces.Box(low = 0, high = 255, shape=[STATE_WIDTH, STATE_HEIGHT, 3], dtype = np.int16)

  def seed(self, seed = None):
    self.np_random, seed = seeding.np_random(seed)

    return [seed]

  # For now, keeping as two separate actions, but may just merge
  def move(self):

    # moves forward x meters and rotates ang radians decided by agent.
    print("\nX, ang: {}, {}".format(float(self.x),float(self.ang))) 
    motionProxy.moveTo(float(self.x), float(0.0), float(self.ang))
    
  def reset(self):
    motionProxy.wakeUp()
    self.reward = 0.0


    videoClient = camProxy.subscribe("python_client", resolution, colorSpace, 5)
    # Get a camera image.
    # image[6] contains the image data passed as an array of ASCII chars.
    naoImage = camProxy.getImageRemote(videoClient)
    camProxy.unsubscribe(videoClient)

    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]

    # only negative reinforcement learns much more slowly - one takeaway

    # Create a PIL Image from our pixel array.
    im = Image.frombytes("RGB", (imageWidth, imageHeight), array)
    im.save("resetEnv.png", "PNG")
    
    im = Image.open("resetEnv.png")
    array = np.array(im)

    redVals = []
    greenVals = []
    blueVals = []
    for i in range(imageWidth):
      redVals.append([x[i][0] for x in array])
      greenVals.append([x[i][1] for x in array])
      blueVals.append([x[i][2] for x in array])

    summyR = np.sum(redVals)
    summyB = np.sum(blueVals)
    summyG = np.sum(greenVals)

    self.prevRed = summyR
    self.prevBlue = summyB
    self.prevGreen = summyG
    self.state = array

    return self.step(None)[0]

  def step(self, action):

    if action is not None:
      #clip to fit action space bounds (can do tanh normalization if linear is too brutish with clipping)

      # action[0] = np.clip(action[0], self.action_space.low[0], self.action_space.high[0])
      # action[1] = np.clip(action[1], self.action_space.low[1], self.action_space.high[1])
      action[0] = action[0]*self.action_space.high[0]
      action[1] = np.clip(action[1]*self.action_space.high[1],  self.action_space.low[1], self.action_space.high[1])
      self.ang = action[0]
      self.x = action[1]
      
    self.move()
      
    videoClient = camProxy.subscribe("python_client", resolution, colorSpace, 5)
    # Get a camera image.
    # image[6] contains the image data passed as an array of ASCII chars.
    naoImage = camProxy.getImageRemote(videoClient)
    camProxy.unsubscribe(videoClient)

    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    
    # Create a PIL Image from our pixel array.
    im = Image.frombytes("RGB", (imageWidth, imageHeight), array)

    # Save the image.
    im.save("camImage.png", "PNG")
    
    im = Image.open("camImage.png")
    array = np.array(im)
    
    #do logic here to determine redness/blueness/greenness of image, is done when the full image is all red/blue/green
    #reward should be a factor of blueness/redness/green
    

    redVals = []
    greenVals = []
    blueVals = []
    for i in range(imageWidth):
      redVals.append([x[i][0] for x in array])
      greenVals.append([x[i][1] for x in array])
      blueVals.append([x[i][2] for x in array])

    summyR = np.sum(redVals)
    summyB = np.sum(blueVals)
    summyG = np.sum(greenVals)

    self.red = summyR
    self.blue = summyB
    self.green = summyG

    self.state = array

    step_reward = 0.0

    done = False

    #done:

    # redDiff = (self.red - self.prevRed)/4800
    # redDiff = float(redDiff)
    # print(str(redDiff))

    # when I redid the simulation, the walls I used were different or maybe not shiny enough so do not increase in red pixels??
    redDiff = (float(self.red) - float(self.prevRed))/4800
    print("redSum: " + str(float(self.red)))
    blueDiff = (float(self.blue) - float(self.prevBlue))/4800

    greenDiff = (float(self.green) - float(self.prevGreen))/4800


    x = 0.0
    y = 0.0
    # if self.red <= 115000:
    #   x = 5.0
    # elif self.red < 100000:
    #   x = 10.0
    # elif self.red > 115000:
    #   y = 15.0
    # elif self.red > 160000:
    #   y = 20.0

    #add reward factor of (redSum - 122500)/50000 or somethin like that 
    # x = (self.red - 120000)/4800

    x = self.red + self.blue + self.green

    if action is not None:
      #discount
      self.reward -= 1
      step_reward = self.reward - self.prev_reward + x/480000
      print "\nstep_reward: " + str(step_reward) + "\n"
      self.prev_reward = self.reward
      if x > 1650000:
        done = True
        motionProxy.rest()
        if self.red > self.blue and self.red > self.green:
          self.reward += 10000.0
        else:
          self.reward -= 5000
        
      
    self.prevRed = self.red
    self.prevBlue = self.blue
    self.prevGreen = self.green


    return self.state, step_reward, done, {}


  def render(self, mode='human'):
    assert mode in ['human', 'state_pixels', 'rgb_array']
    return 

  def close(self):
    motionProxy.rest()
    

