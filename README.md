## Inspector Baxter: Speech Recognition Node
#### ME 495: Embedded Systems in Robotics
#### _Aaron Weatherly_

## Basic Overview
This allows a user to control Baxter's operating state through speech recognition

## Instructions
Follow tutorials [here][baxter_tutorials] to set up your workstation and connect to Baxter.
Install the `pocketsphinx` package by following the instructions [here][pocketsphinx].
Also, make sure your workstation has OpenCV installed: `sudo apt-get install python-opencv`.

Now we need a ROS node that listens for specific keywords from the user and updates Baxter's 
current operating state accordingly. 
```bash
cd ~/catkin_ws/src
catkin_create_pkg baxter_speech rospy std_msgs sensor_msgs
cd ..
catkin_make
```

To identify the keywords we will be using, we need to copy the `vocab` directory from the 
`pocketsphinx` package into this node's directory:
```bash
cp ~/catkin_ws/src/pocketsphinx/vocab/* ~/catkin_ws/src/baxter_speech/vocab
```

Two files here need to be edited. [voice_cmd.dic](vocab/voice_cmd.dic) is the dictionary file, 
a list of words and the correct pronunciation for them. See the [CMU Pronouncing Dictionary][cmu] 
for reference. [voice_cmd.kwlist](vocab/voice_cmd.kwlist) is the list of keywords or phrases that 
pocketsphinx listens for before publishing. Phrases can consist of multiple words, as long as they 
are all defined in the dictionary file. Whenever a keyword is heard, `pocketsphinx` will publish 
it as a string to the `/pocketsphinx_recognizer/output` topic.

Our node, [baxter_speech.py](src/baxter_speech.py) subscribes to the `/pocketsphinx_recognizer/output`
topic and then publishes to the `/state` topic based on what the user has said. For debugging,
the node also publishes contextual text/images to baxter's screen on the `/robot/xdisplay` topic.

The launch file, [baxter_speech.launch](launch/baxter_speech.launch) runs both the `pocketsphinx` 
and the `baxter_speech` nodes and defines the location of the dictionary and keyword files. To run 
this, connect to and enable Baxter and then use: `roslaunch baxter_speech baxter_speech.launch`
If not connected to the actual robot, you can view the images that would display on Baxter's screen 
by running `rosrun image_view image_view image:=/robot/xdisplay` in a separate terminal.

Now, as the user gives commands, our other nodes can subscribe to the `/state` topic and do object 
identification and manipulation!


[baxter_tutorials]: http://sdk.rethinkrobotics.com/wiki/Baxter_Setup
[pocketsphinx]: https://github.com/UTNuclearRoboticsPublic/pocketsphinx
[cmu]: http://www.speech.cs.cmu.edu/cgi-bin/cmudict

