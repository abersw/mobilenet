#!/usr/bin/env python
#To do:
#Get room name for file write
from __future__ import print_function

import roslib
import os, sys
import rospy, rospkg
import cv2
import array
from wheelchair_msgs.msg import mobilenet #import the wheelchair messages files
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

objectLog = [] # logs lists of objects when they are detected by MNETv2
objectConfLog = [] # logs list of objects confidence

objectList = [] # list of signle items found by MNETv2
objectConfList = [] # list of confidence for single items

objectString = "";



itemInLog = ""
itemInList = ""

model = cv2.dnn.readNetFromTensorflow('/home/tomos/ros/wheelchair/catkin_ws/src/mobilenet/scripts/models/frozen_inference_graph.pb',
                                        '/home/tomos/ros/wheelchair/catkin_ws/src/mobilenet/scripts/models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

frameCount = 0

def addFrame():
  global frameCount
  frameCount = frameCount + 1
  #print("framecount: ", frameCount) #this is the frame number

classNames = {0: 'background',
              1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane', 6: 'bus',
              7: 'train', 8: 'truck', 9: 'boat', 10: 'traffic light', 11: 'fire hydrant',
              13: 'stop sign', 14: 'parking meter', 15: 'bench', 16: 'bird', 17: 'cat',
              18: 'dog', 19: 'horse', 20: 'sheep', 21: 'cow', 22: 'elephant', 23: 'bear',
              24: 'zebra', 25: 'giraffe', 27: 'backpack', 28: 'umbrella', 31: 'handbag',
              32: 'tie', 33: 'suitcase', 34: 'frisbee', 35: 'skis', 36: 'snowboard',
              37: 'sports ball', 38: 'kite', 39: 'baseball bat', 40: 'baseball glove',
              41: 'skateboard', 42: 'surfboard', 43: 'tennis racket', 44: 'bottle',
              46: 'wine glass', 47: 'cup', 48: 'fork', 49: 'knife', 50: 'spoon',
              51: 'bowl', 52: 'banana', 53: 'apple', 54: 'sandwich', 55: 'orange',
              56: 'broccoli', 57: 'carrot', 58: 'hot dog', 59: 'pizza', 60: 'donut',
              61: 'cake', 62: 'chair', 63: 'couch', 64: 'potted plant', 65: 'bed',
              67: 'dining table', 70: 'toilet', 72: 'tv', 73: 'laptop', 74: 'mouse',
              75: 'remote', 76: 'keyboard', 77: 'cell phone', 78: 'microwave', 79: 'oven',
              80: 'toaster', 81: 'sink', 82: 'refrigerator', 84: 'book', 85: 'clock',
              86: 'vase', 87: 'scissors', 88: 'teddy bear', 89: 'hair drier', 90: 'toothbrush'}



def id_class_name(class_id, classes):
  for key, value in classes.items():
    if class_id == key:
      return value

class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    mobilenet_src = rospy.get_param("/param/mobilenet/image_src") #get camera topic from ROS param server

    self.image_sub = rospy.Subscriber(mobilenet_src, Image, self.callback) #rosparam camera source

    self.pub_annotated_image = rospy.Publisher("/wheelchair_robot/mobilenet/annotated_image",Image, queue_size=10) #publish annotated image

    self.pub_detected_object = rospy.Publisher("/wheelchair_robot/mobilenet/detected_object",MultiArrayDimension, queue_size=20)
    self.pub_image = rospy.Publisher("/wheelchair_robot/mobilenet/raw_image", Image, queue_size=10)
    self.pub_object_name = rospy.Publisher("/wheelchair_robot/mobilenet/object_name",String, queue_size=10)
    #self.pub_object_confidence = rospy.Publisher("/wheelchair_robot/mobilenet/object_confidence",Float32, queue_size=10)
    #self.pub_box_x = rospy.Publisher("/wheelchair_robot/mobilenet/box_x",Float32, queue_size=10)
    #self.pub_box_y = rospy.Publisher("/wheelchair_robot/mobilenet/box_y",Float32, queue_size=10)
    #self.pub_box_width = rospy.Publisher("/wheelchair_robot/mobilenet/box_width",Float32, queue_size=10)
    #self.pub_box_height = rospy.Publisher("/wheelchair_robot/mobilenet/box_height",Float32, queue_size=10)



  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      obj = String()
      addFrame()
      mobilenet_confidence_threshold = rospy.get_param("/param/mobilenet/confidence_threshold")
    except CvBridgeError as e:
      print(e)

    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)

    #cv2.imshow("Image window", cv_image)
    c = cv2.waitKey(1)
    if c == 27: #this is the escape key
      cv2.destroyAllWindows()

    #add code for object classification
    # Loading model
    #model = cv2.dnn.readNetFromTensorflow('models/frozen_inference_graph.pb',
    #                                      'models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')
    #image = cv2.imread("generate.jpg")

    image = cv2.resize(cv_image, (0,0), fx=1.0, fy=1.0)

    image_height, image_width, _ = image.shape

    model.setInput(cv2.dnn.blobFromImage(image, size=(300, 300), swapRB=True))
    output = model.forward()

    #objectList = String()

    for detection in output[0, 0, :, :]:
        confidence = detection[2]
        if confidence > mobilenet_confidence_threshold:
            class_id = detection[1]
            class_name=id_class_name(class_id,classNames)
            print(str(str(class_id) + " " + str(detection[2])  + " " + class_name))
            #add object logger here
            objectLog.append(class_name)
            objectConfLog.append(confidence)
            box_x = detection[3] * image_width
            box_y = detection[4] * image_height
            box_width = detection[5] * image_width
            box_height = detection[6] * image_height
            cv2.rectangle(image, (int(box_x), int(box_y)), (int(box_width), int(box_height)), (23, 230, 210), thickness=1)
            cv2.putText(image,class_name ,(int(box_x), int(box_y+.05*image_height)),cv2.FONT_HERSHEY_SIMPLEX,(.002*image_width),(0, 0, 255))
            #class_name_array += class_name
            #frameCount = frameCount + 1
            #objectList.join(class_name)
    #obj = objectList
    #cv2.imshow('image', image)





    #cv2.waitKey(3)

    try:
      self.pub_annotated_image.publish(self.bridge.cv2_to_imgmsg(image, "bgr8")) #publish annotated image

      self.pub_image.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8")) #publish raw image
      #self.pub_detected_object.publish(obj)
      #self.pub_object_name.publish(String(obj))
    except CvBridgeError as e:
      print(e)

def foundObjectsFileWrite():
  #new section saves the varience of objects into a txt file "found-objects.txt"
    # record object type in a file - 1 file per training - overwrite when finished.
    roomNameParam = rospy.get_param("/wheelchair_robot/user/room_name") #get room name from user
    rospack = rospkg.RosPack()
    foundInstanceFlag = 0
    bagOfObjects = open(os.path.join(rospack.get_path("wheelchair_dump"), "dump", "mobilenet", roomNameParam + ".objects"), 'w') #location of dump package
    print(objectLog) #print off raw list
    posInObjectLog = 0
    for itemInLog in objectLog: #iterate through items in log
        print(itemInLog)
        for itemInList in objectList: #iterate through items in found list
            if itemInLog == itemInList: #if log item is same as found item
                foundInstanceFlag += 1 #add 1 instance
        if foundInstanceFlag == 0: #if we haven't found an instance of an object
            objectList.append(itemInLog) #append object name to array
            objectConfList.append(objectConfLog[posInObjectLog]) # get confidence of corresponding object
        foundInstanceFlag = 0 #set back to 0 when finished
        posInObjectLog += 1

    print(objectList) #print off instance list array
    posInObjectList = 0
    for itemInList in objectList: #iterate through instance list
        concatToBag = itemInList + ":" + str(objectConfList[posInObjectList]) + "\n";
        print(concatToBag)
        bagOfObjects.write(concatToBag) #write object instance to file
        posInObjectList += 1
    bagOfObjects.close()

def main(args):
  ic = image_converter()
  rospy.init_node('mobilenet_ROSit_2_OCV', anonymous=True)
  try:
    rospy.spin()
    print("Shutting down")
    foundObjectsFileWrite()


  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
