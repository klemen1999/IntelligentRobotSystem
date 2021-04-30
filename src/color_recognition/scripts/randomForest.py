#! /usr/bin/env python

import os
from PIL import Image
import re
import numpy as np
from skimage.color import rgb2hsv
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn import metrics
import pickle

import rospy
from ring_detection.msg import Int2dArray, IntList
from color_recognition.msg import PoseColor



class RandomForest:
    def __init__(self):
        rospy.init_node('random_forest', anonymous=True)
        self.CLASSES = {"white": 0, "black": 1, "red": 2, "blue": 3, "green": 4, "yellow": 5}

        self.ring_rgb_list_sub = rospy.Subscriber("ring_color_pub", Int2dArray, self.ring_callback)
        base_dir = os.path.dirname(os.path.realpath(__file__))

        filename = '/color_model_RF.sav'
        self.loaded_model = pickle.load(open(base_dir + filename, 'rb'))

        self.pose_color_ring_pub = rospy.Publisher('ring_pose', PoseColor, queue_size=1000)

        
    
    def ring_callback(self, data):
        colors = data.lists
        np_colors = np.empty((len(colors), 3))
        for i in range(len(colors)):
            np_colors[i] = colors[i].elements
        rgb_hsv_data = self.getImageData(np_colors)
        col = self.predict(rgb_hsv_data)
        
        msg = PoseColor()
        msg.pose = data.pose
        msg.color = col
        #publish to markers (pose and color)
        self.pose_color_ring_pub.publish(msg)

    def predict(self, colors):
        result = self.loaded_model.predict(colors)
        counts = np.bincount(result.astype(int))
        index = np.argmax(counts)

        val_list = list(self.CLASSES.values())
        key_list = list(self.CLASSES.keys())
        position = val_list.index(index)


        #print(key_list[position])
        return key_list[position]

    def getImageData(self, image):
        data = np.empty((len(image), 6))
        imageHSV = rgb2hsv(image)
        counter = 0
        for i in range(len(image)):
            rgb = image[i, :]
            hsv = imageHSV[i, :]
            data[counter, :] = np.concatenate((rgb, hsv))
            counter += 1
        return data



if __name__ == "__main__":
    colorDetector = RandomForest()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
