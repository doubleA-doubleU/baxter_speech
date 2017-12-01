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
import time

from sensor_msgs.msg import (
    Image,
)
from std_msgs.msg import String
from inspector.msg import ObjectName
from inspector.msg import State

# define the states
STATE_INIT    = 0
STATE_TRAIN   = 1
STATE_SORT    = 2
STATE_FETCH   = 3
STATE_EXIT    = 4
STATE_FINISH  = 5
STATE_LISTEN  = 8
STATE_NEXT    = 10
    
go = -1 # represents whether state changes are acceptable
state = 0
name = "unknown"

def send_image(path):
    img = cv2.imread(path) # use OpenCV to read the image file
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8") # convert file to ROS image
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1) # define publisher
    pub.publish(msg) # publish the image to Baxter's screen on the /robot/xdisplay topic

def state_change(msg):  
    global go
    global state
    global name

    # define the publishers
    state_pub = rospy.Publisher('/inspector/state',State,queue_size=1)
    name_pub = rospy.Publisher('/inspector/naming',ObjectName,queue_size=1)
    
    # base directory for this package
    BASE_DIR = os.path.join( os.path.dirname( __file__ ), '..' )

    if (go==-1):
        if (msg.data=="baxter "):
            state = STATE_INIT
            time.sleep(1) # add a delay so the topic initial state can be published
            state_pub.publish(state) # publish the current state to the /inspector/state topic
            file = "images/init.png"
            send_image(os.path.join(BASE_DIR,file)) # update image on Baxter's screen  
            go = 1
        
    elif (go==1): # only change states if other nodes have completed a phase
        # change state and image file based on predefined keywords    
        if (msg.data=="baxter "):
            file = "images/baxter.png"
            state = STATE_LISTEN
            state_pub.publish(state) # publish the current state to the /inspector/state topic
            send_image(os.path.join(BASE_DIR,file)) # update image on Baxter's screen
        elif (state==STATE_LISTEN):
            if (msg.data=="baxter "):
                file = "images/baxter.png"
                state = STATE_LISTEN
            elif (msg.data=="learn "):
                file = "images/learn.png"
                state = STATE_TRAIN
            elif (msg.data=="fetch "):
                file = "images/fetch.png"
                state = STATE_FETCH
            elif (msg.data=="sort "):
                file = "images/sort.png"
                state = STATE_SORT
                go = 0
            elif (msg.data=="shut down " or msg.data=="exit "):
                file = "images/exit.png"
                state = STATE_EXIT
                go = 0
            else:
                file = "images/error.png"
                
            state_pub.publish(state) # publish the current state to the /inspector/state topic
            send_image(os.path.join(BASE_DIR,file)) # update image on Baxter's screen  
            
        elif (state==STATE_TRAIN):
            if (msg.data=="can "):
                file = "images/learn_can.png"
                name = "can" 
                go = 0
            elif (msg.data=="bottle "):
                file = "images/learn_bottle.png"
                name = "bottle"
                go = 0
            elif (msg.data=="cube "):
                file = "images/learn_cube.png"
                name = "cube"
                go = 0
            elif (msg.data=="baxter "):
                file = "images/baxter.png"
                name = "unknown"
            elif (msg.data=="learn "):
                file = "images/learn.png"
                name = "unknown"
            else:
                file = "images/error.png"
                name = "unknown"
            
            state_pub.publish(state)
            name_pub.publish(name) # publish the current object name to the /inspector/naming topic
            send_image(os.path.join(BASE_DIR,file)) # update image on Baxter's screen
            
        elif (state==STATE_FETCH):
            if (msg.data=="can "):
                file = "images/fetch_can.png"
                name = "can"
                go = 0
            elif (msg.data=="bottle "):
                file = "images/fetch_bottle.png"
                name = "bottle"
                go = 0
            elif (msg.data=="cube "):
                file = "images/fetch_cube.png"
                name = "cube"
                go = 0
            elif (msg.data=="baxter "):
                file = "images/baxter.png"
                name = "unknown"
            elif (msg.data=="fetch "):
                file = "images/fetch.png"
                name = "unknown"
            else:
                file = "images/error.png"
                name = "unknown"
            
            state_pub.publish(state)
            name_pub.publish(name) # publish the current object name to the /inspector/naming topic
            send_image(os.path.join(BASE_DIR,file)) # update image on Baxter's screen
        
def next_state(msg):
    global go
    if (msg.state==STATE_NEXT):
        go = 1

def main():
    rospy.init_node('baxter_speech') # initialize node
    rospy.Subscriber("/pocketsphinx_recognizer/output",String,state_change) # define subscriber
    rospy.Subscriber("/inspector/state",State,next_state) # define subscriber
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting Down")