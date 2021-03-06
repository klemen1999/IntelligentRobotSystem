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
from color_recognition.msg import Int2dArray, IntList
from color_recognition.srv import ObjectColor, ObjectColorResponse



class RandomForest:
    def __init__(self):
        rospy.init_node('random_forest')
        self.CLASSES = {"white": 0, "black": 1, "red": 2, "blue": 3, "green": 4, "yellow": 5}

        # service to get ring color
        self.ring_color_srv = rospy.Service("ring_color", ObjectColor, self.color_callback)

        # service to get cylinder color
        self.cylinder_color_srv = rospy.Service("cylinder_color", ObjectColor, self.color_callback)

        base_dir = os.path.dirname(os.path.realpath(__file__))

        filename = '/color_model_RF.sav'
        self.loaded_model = pickle.load(open(base_dir + filename, 'rb'))
        

    def color_callback(self, request):
        print("New color requst")
        colors = request.data.lists
        np_colors = np.empty((len(colors), 3))
        for i in range(len(colors)):
            np_colors[i] = colors[i].elements
        rgb_hsv_data = self.getImageData(np_colors)
        col = self.predict(rgb_hsv_data)
        
        msg = ObjectColorResponse()
        msg.color = col
        print("Response: ", col)
        return msg

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
