# -*- encoding: UTF-8 -*-

"""
This example shows how to use ALTracker with red ball and LArm.
"""

import time
import argparse
from PIL import Image
from naoqi import ALProxy

def main(IP, PORT, ballSize, effector):

    print "Connecting to", IP, "with port", PORT

    motion = ALProxy("ALMotion", IP, PORT)
    posture = ALProxy("ALRobotPosture", IP, PORT)
    tracker = ALProxy("ALTracker", IP, PORT)
    camProxy = ALProxy("ALVideoDevice", IP, PORT)
    resolution = 2    # VGA
    colorSpace = 11   # RGB

    videoClient = camProxy.subscribe("python_client", resolution, colorSpace, 5)

    # First, wake up.
    motion.wakeUp()

    fractionMaxSpeed = 0.8
    # Go to posture stand
    posture.goToPosture("StandInit", fractionMaxSpeed)

    # Add target to track.
    targetName = "RedBall"
    diameterOfBall = ballSize
    tracker.registerTarget(targetName, diameterOfBall)

    # set mode
    mode = "Head"
    tracker.setMode(mode)

    # set effector
    tracker.setEffector(effector)

    # Then, start tracker.
    tracker.track(targetName)

    print "ALTracker successfully started, now show a red ball to robot!"
    print "Use Ctrl+c to stop this script."
    
    print "Torso Frame Target Position: "
    print tracker.getTargetPosition(0)
    print "World Frame Target Position: "
    print tracker.getTargetPosition(1)
    print "Robot Frame Target Position: "
    print tracker.getTargetPosition(2)

    tracker.lookAt(tracker.getTargetPosition(2), 2, 0.5, 0)

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
    im = Image.frombytes("RGB", (imageWidth, imageHeight), array)

    # Save the image.
    im.save("trackerImage.png", "PNG")
    
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print
        print "Interrupted by user"
        print "Stopping..."

    # Stop tracker, go to posture Sit.
    tracker.stopTracker()
    tracker.unregisterAllTargets()
    tracker.setEffector("None")
    posture.goToPosture("Sit", fractionMaxSpeed)
    motion.rest()

    print "ALTracker stopped."


if __name__ == "__main__" :

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.137.238",
                        help="Robot ip address.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number.")
    parser.add_argument("--ballsize", type=float, default=0.06,
                        help="Diameter of ball.")
    parser.add_argument("--effector", type=str, default="LArm",
                        choices=["Arms", "LArm", "RArm", "Head"],
                        help="Effector for tracking.")

    args = parser.parse_args()

    main(args.ip, args.port, args.ballsize, args.effector)
