#! /usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from sound.msg import RobotSpeakRequest


class Sound_Controller:

    def __init__(self):
        rospy.init_node("robot_say_node", anonymous = False)
        self.sound_client = SoundClient()
        self.sound_client.stopAll()
        self.sub = rospy.Subscriber("/robot_say", RobotSpeakRequest, self.robot_say_listener)
        rospy.spin()

    def robot_say_listener(self, msg):
        sentence = msg.message
        print(f"Recieved a message {sentence}")
        self.robot_say(sentence)

    def robot_say(self, sentence):
        self.sound_client.say(sentence)
        rospy.sleep(3)

    
if __name__ == '__main__':
    sound_controller = Sound_Controller()
        