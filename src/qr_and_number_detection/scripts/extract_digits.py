#!/usr/bin/python

import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import ColorRGBA
from qr_and_number_detection.msg import DigitsMessage
import pytesseract

dictm = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# The object that we will pass to the markerDetect function
params =  cv2.aruco.DetectorParameters_create()

print(params.adaptiveThreshConstant) 
print(params.adaptiveThreshWinSizeMax)
print(params.adaptiveThreshWinSizeMin)
print(params.minCornerDistanceRate)
print(params.adaptiveThreshWinSizeStep)

# To see description of the parameters
# https://docs.opencv.org/3.3.1/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html

# You can set these parameters to get better marker detections
params.adaptiveThreshConstant = 25
params.adaptiveThreshWinSizeStep = 2


class DigitExtractor:
    def __init__(self):
        rospy.init_node('image_converter', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # Subscribe to the image and/or depth topic
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        #Publisher for the extracted digits
        self.digits_pub = rospy.Publisher("/digits", DigitsMessage, queue_size=100)


    def image_callback(self,data):
        # print('Iam here!')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        print("Will try to process the image...")

        corners, ids, rejected_corners = cv2.aruco.detectMarkers(cv_image,dictm,parameters=params)
        
        # Increase proportionally if you want a larger image
        image_size=(351,248,3)
        marker_side=50

        img_out = np.zeros(image_size, np.uint8)
        out_pts = np.array([[marker_side/2,img_out.shape[0]-marker_side/2],
                        [img_out.shape[1]-marker_side/2,img_out.shape[0]-marker_side/2],
                        [marker_side/2,marker_side/2],
                        [img_out.shape[1]-marker_side/2,marker_side/2]])

        src_points = np.zeros((4,2))
        cens_mars = np.zeros((4,2))

        if not ids is None:
            if len(ids)==4:
                rospy.loginfo('4 Markers detected')
                print(str(set(ids.ravel())))
                # check that it did not detect two markers with the same id
                if not len(set(ids.ravel())) == len(ids):
                    #print("not unique")
                    return

                for idx in ids:
                    # Calculate the center point of all markers
                    cors = np.squeeze(corners[idx[0]-1])
                    cen_mar = np.mean(cors,axis=0)
                    cens_mars[idx[0]-1]=cen_mar
                    cen_point = np.mean(cens_mars,axis=0)
                    #print("id: " + str(idx[0]))
                    #print(str(cen_mar))
                    #print(cens_mars)
                # check if id 3 and 4 are to the right of id 1 and 2
                # if (cens_mars[2][1] > cens_mars[0][1] or
                #     cens_mars[2][1] > cens_mars[1][1] or
                #     cens_mars[3][1] > cens_mars[0][1] or
                #     cens_mars[3][1] > cens_mars[1][1]):
                #     #print("nopeeeeeeeee")
                #     return
                
                for coords in cens_mars:
                    #  Map the correct source points
                    if coords[0]<cen_point[0] and coords[1]<cen_point[1]:
                        src_points[2]=coords
                    elif coords[0]<cen_point[0] and coords[1]>cen_point[1]:
                        src_points[0]=coords
                    elif coords[0]>cen_point[0] and coords[1]<cen_point[1]:
                        src_points[3]=coords
                    else:
                        src_points[1]=coords

                h, status = cv2.findHomography(src_points, out_pts)
                img_out = cv2.warpPerspective(cv_image, h, (img_out.shape[1],img_out.shape[0]))
                ################################################
                #### Extraction of digits starts here
                ################################################
                
                # Cut out everything but the numbers
                img_out = img_out[125:221,50:195,:]
                
                # Convert the image to grayscale
                img_out = cv2.cvtColor(img_out, cv2.COLOR_BGR2GRAY)
                
                # Option 1 - use ordinairy threshold the image to get a black and white image
                #ret,img_out = cv2.threshold(img_out,100,255,0)

                # Option 1 - use adaptive thresholding
                img_out = cv2.adaptiveThreshold(img_out,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,5)
                
                # Use Otsu's thresholding
                #ret,img_out = cv2.threshold(img_out,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                
                # Pass some options to tesseract
                config = '--psm 13 outputbase nobatch digits'
                
                # Visualize the image we are passing to Tesseract
                cv2.imshow('Warped image',img_out)
                cv2.waitKey(1)
            
                # Extract text from image
                text = pytesseract.image_to_string(img_out, config = config)
                
                # Check and extract data from text
                print('Extracted>>',text)
                
                # Remove any whitespaces from the left and right
                text = text.strip()
                
                # If the extracted text is of the right length
                if len(text)==2:
                    x=int(text[0])
                    y=int(text[1])
                    message = DigitsMessage()
                    message.first_digit = x
                    message.second_digit = y
                    self.digits_pub.publish(message)
                    print('The extracted datapoints are x='+str(x)+', y='+str(y))
                else:
                    print('The extracted text has is of length '+str(len(text))+'. Aborting processing')
            # elif len(ids)==3:
            #     rospy.loginfo('3 Markers detected')
            #     #print(len(ids))
            #     #print(corners)
            #     min_max_x = [cv_image.shape[0], 0]
            #     min_max_y = [cv_image.shape[1], 0]
            #     #print("IDS " + str(ids))
            #     # check that it did not detect two markers with the same id
            #     if not len(set(ids.ravel())) == len(ids):
            #         #print("not unique")
            #         return
            #     for idx in ids:
            #         # Calculate the center point of all markers
            #         if len(corners) < idx[0]:
            #             cors = np.squeeze(corners[idx[0]-2])
            #         else:
            #             cors = np.squeeze(corners[idx[0]-1])
            #         cen_mar = np.mean(cors,axis=0)
            #         if len(corners) < idx[0]:
            #             cens_mars[idx[0]-2]=cen_mar
            #         else:
            #             cens_mars[idx[0]-1]=cen_mar
            #         #print(cens_mars)
            #         if cen_mar[0] < min_max_x[0]:
            #             min_max_x[0] = cen_mar[0]
            #         if cen_mar[0] > min_max_x[1]:
            #             min_max_x[1] = cen_mar[0]
            #         if cen_mar[1] < min_max_y[0]:
            #             min_max_y[0] = cen_mar[1]
            #         if cen_mar[1] > min_max_y[1]:
            #             min_max_y[1] = cen_mar[1]
            #         cen_point = np.mean(cens_mars,axis=0)
            #     #print(min_max_x)
            #     #print(min_max_y)he right of ids 1 and 2
            #     # check if ids 3 and 4 are to t
            #     if (
            #         (
            #         (set(ids.ravel()) == {1,2,3} or set(ids.ravel()) == {1,2,4}) 
            #         and (cens_mars[2][1] > cens_mars[0][1] or cens_mars[2][1] > cens_mars[1][1])
            #         )
            #         or
            #         (
            #             (set(ids.ravel()) == {1,3,4} or set(ids.ravel()) == {2,3,4}) and
            #             (
            #                 cens_mars[1][1] > cens_mars[0][1] or
            #                 cens_mars[2][1] > cens_mars[0][1]
            #             )
            #         )
            #         ):
            #         #print("nopeeeeeeeee")
            #         return

            #     for coords in cens_mars:
            #         #  Map the correct source points
            #         if coords[0]<cen_point[0] and coords[1]<cen_point[1]:
            #             src_points[2]=coords
            #         elif coords[0]<cen_point[0] and coords[1]>cen_point[1]:
            #             src_points[0]=coords
            #         elif coords[0]>cen_point[0] and coords[1]<cen_point[1]:
            #             src_points[3]=coords
            #         else:
            #             src_points[1]=coords
            #     #h, status = cv2.findHomography(src_points, out_pts)
            #     #img_out = cv2.warpPerspective(cv_image, h, (img_out.shape[1],img_out.shape[0]))
            #     #print(str(cv_image.shape))
            #     #img_out = cv_image[int(min_max_x[0]):int(min_max_x[1]), int(min_max_y[0]):int(min_max_y[1])]
            #     #print(cv_image.shape[1])
            #     #print(int(min_max_y[0]))
            #     #print(int(min_max_y[1]))
            #     razlika_y = min_max_y[1] - min_max_y[0]
            #     razlika_x = min_max_x[1] - min_max_x[0]
            #     img_out = cv_image[int(min_max_y[0]+razlika_y*0.1):int(min_max_y[1]-razlika_y*0.1), int(min_max_x[0]+razlika_x*0.1):int(min_max_x[1]-razlika_x*0.1)]
            #     ################################################
            #     #### Extraction of digits starts here
            #     ################################################
                
            #     # Cut out everything but the numbers
            #     #img_out = img_out[125:221,50:195,:]
                
            #     # Convert the image to grayscale
            #     img_out = cv2.cvtColor(img_out, cv2.COLOR_BGR2GRAY)
                
            #     # Option 1 - use ordinairy threshold the image to get a black and white image
            #     #ret,img_out = cv2.threshold(img_out,100,255,0)

            #     # Option 1 - use adaptive thresholding
            #     img_out = cv2.adaptiveThreshold(img_out,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,5)
                
            #     # Use Otsu's thresholding
            #     #ret,img_out = cv2.threshold(img_out,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                
            #     # Pass some options to tesseract
            #     config = '--psm 13 outputbase nobatch digits'
                
            #     # Visualize the image we are passing to Tesseract
            #     cv2.imshow('Warped image',img_out)
            #     cv2.waitKey(1)
            
            #     # Extract text from image
            #     text = pytesseract.image_to_string(img_out, config = config)
                
            #     # Check and extract data from text
            #     print('Extracted>>',text)
                
            #     # Remove any whitespaces from the left and right
            #     text = text.strip()
                
            #     # If the extracted text is of the right length
            #     if len(text)==2:
            #         x=int(text[0])
            #         y=int(text[1])
            #         message = DigitsMessage()
            #         message.first_digit = x
            #         message.second_digit = y
            #         self.digits_pub.publish(message)
            #         print('The extracted datapoints are x='+str(x)+', y='+str(y))
            #     else:
            #         print('The extracted text has is of length '+str(len(text))+'. Aborting processing')

            elif len(ids) >= 5 and len(ids) <= 8:
                print('got ' + str(len(ids)) + ' markers')
                #get centers of all markers
                centers = []
                i = 0
                remember = {}
                for idx in ids:
                    # Calculate the center point of all markers
                    cors = np.squeeze(corners[i])
                    cen_mar = np.mean(cors,axis=0)
                    centers.append(cen_mar)
                    i += 1
                    # remember marker id and center
                    remember[i] = {}
                    remember[i]["id"] = idx[0]
                    remember[i]["cen_mar"] = cen_mar.tolist()
                
                #get 4 most right markers
                most_right = np.sort(centers, 1)[-4:]
                # check which marker ids are choosen
                all_ids = []
                for c in remember:
                    d = remember[c]
                    for mr in most_right:
                        if d["cen_mar"][0] == mr[1] and d["cen_mar"][1] == mr[0]:
                            all_ids.append(d["id"])
                # non unique markers choosen
                if not len(set(all_ids)) == len(all_ids):
                    return
                # get the coordinates of edges
                min_max_x = [cv_image.shape[0], 0]
                min_max_y = [cv_image.shape[1], 0]
                
                min_max_x[0] = min(most_right, key=lambda x: x[0])[0]
                min_max_y[0] = min(most_right, key=lambda x: x[1])[1]

                min_max_x[1] = max(most_right, key=lambda x: x[0])[0]
                min_max_y[1] = max(most_right, key=lambda x: x[1])[1]
                
                # cut 10% of edges to get a more clear picture
                razlika_y = min_max_y[1] - min_max_y[0]
                razlika_x = min_max_x[1] - min_max_x[0]
                img_out = cv_image[int(min_max_x[0]+razlika_x*0.1):int(min_max_x[1]-razlika_x*0.1), int(min_max_y[0]+razlika_y*0.1):int(min_max_y[1]-razlika_y*0.1)]
                ################################################
                #### Extraction of digits starts here
                ################################################
                
                # Cut out everything but the numbers
                #img_out = img_out[125:221,50:195,:]
                
                # Convert the image to grayscale
                img_out = cv2.cvtColor(img_out, cv2.COLOR_BGR2GRAY)
                
                # Option 1 - use ordinairy threshold the image to get a black and white image
                #ret,img_out = cv2.threshold(img_out,100,255,0)

                # Option 1 - use adaptive thresholding
                img_out = cv2.adaptiveThreshold(img_out,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,5)
                
                # Use Otsu's thresholding
                #ret,img_out = cv2.threshold(img_out,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                
                # Pass some options to tesseract
                config = '--psm 13 outputbase nobatch digits'
                
                # Visualize the image we are passing to Tesseract
                cv2.imshow('Warped image',img_out)
                cv2.waitKey(1)
            
                # Extract text from image
                text = pytesseract.image_to_string(img_out, config = config)
                
                # Check and extract data from text
                print('Extracted>>',text)
                
                # Remove any whitespaces from the left and right
                text = text.strip()
                
                # If the extracted text is of the right length
                if len(text)==2:
                    x=int(text[0])
                    y=int(text[1])
                    message = DigitsMessage()
                    message.first_digit = x
                    message.second_digit = y
                    self.digits_pub.publish(message)
                    print('The extracted datapoints are x='+str(x)+', y='+str(y))
                else:
                    print('The extracted text has is of length '+str(len(text))+'. Aborting processing')
                
            else:
                print('The number of markers is not ok:',str(len(ids)))
        else:
            print('No markers found')


def main(args):

    de = DigitExtractor()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
