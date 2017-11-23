#!/usr/bin/env python

#####################################################################################
## This is a ROS node that subscribes to the /pocketsphinx_recognizer/output topic ##
## and changes/publishes a state message for the Baxter robot. Contextual images   ##
## are also published to the /robot/xdisplay topic based on Baxter's state.        ##
##                                                                                 ##
## References:                                                                     ##
##    http://sdk.rethinkrobotics.com/wiki/Display_Image_Example                    ##
##    https://github.com/UTNuclearRoboticsPublic/pocketsphinx                      ##
##    http://www.speech.cs.cmu.edu/cgi-bin/cmudict                                 ##
#####################################################################################

import os
import rospy
import cv2
import cv_bridge

from sensor_msgs.msg import (
    Image,
)
from std_msgs.msg import String


def send_image(path):
    img = cv2.imread(path) # use OpenCV to read the image file
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8") # convert file to ROS image
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1) # define publisher
    pub.publish(msg) # publish the image to Baxter's screen on the /robot/xdisplay topic

def state_change(msg):    
    state_pub = rospy.Publisher('state',String,queue_size=1) # define publisher
    # change state and image file based on predefined keywords
    if (msg.data=="baxter "):
        file = "images/baxter.bmp"
        state = "listening"
    elif (msg.data=="learn "):
        file = "images/learn.bmp"
        state = "learning"
    elif (msg.data=="fetch "):
        file = "images/fetch.bmp"
        state = "fetching"
    elif (msg.data=="sort "):
        file = "images/sort.bmp"
        state = "sorting"
    elif (msg.data=="sleep "):
        file = "images/sleep.bmp"
        state = "sleeping"
    else:
        if (msg.data=="can "):
            file = "images/can.bmp"
            state = "identifying"
        elif (msg.data=="bottle "):
            file = "images/bottle.bmp"
            state = "identifying"
        elif (msg.data=="cube "):
            file = "images/cube.bmp"
            state = "identifying"
        else:
            file = "images/error.bmp"
            state = "error"
    
    state_pub.publish(state) # publish the current state to the /baxter_speech/state topic
    
    BASE_DIR = os.path.join( os.path.dirname( __file__ ), '..' ) # base directory for this package
    send_image(os.path.join(BASE_DIR,file)) # update image on Baxter's screen
    
def main():
    rospy.init_node('baxter_speech') # initialize node
    rospy.Subscriber("/pocketsphinx_recognizer/output",String,state_change) # define subscriber
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting Down")