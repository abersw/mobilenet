#!/usr/bin/env python3
from __future__ import print_function

import roslib
import os, sys
import rospy, rospkg
import cv2
from wheelchair_msgs.msg import mobilenet #import the wheelchair messages files
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from time import sleep

objectLog = [] # logs lists of objects when they are detected by MNETv2
objectConfLog = [] # logs list of objects confidence

objectList = [] # list of signle items found by MNETv2
objectConfList = [] # list of confidence for single items

DEBUG_printObjectDetected = 0
DEBUG_noObjectsInFrame = 0

itemInLog = ""
itemInList = ""

model = cv2.dnn.readNetFromTensorflow('/home/tomos/ros/wheelchair/catkin_ws/src/mobilenet/scripts/models/frozen_inference_graph.pb',
                                        '/home/tomos/ros/wheelchair/catkin_ws/src/mobilenet/scripts/models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

#model = cv2.dnn.readNetFromTensorflow('/home/tomos/ros/wheelchair/catkin_ws/src/mobilenet/scripts/models/resnet/frozen_inference_graph.pb',
#                                      '/home/tomos/ros/wheelchair/catkin_ws/src/mobilenet/scripts/models/resnet/faster_rcnn_resnet50_coco_2018_01_28.pbtxt')

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
    rospy.init_node('mobilenet', anonymous=False)

    self.bridge = CvBridge()
    mobilenet_src = rospy.get_param("/wheelchair_robot/param/left_camera") #get camera topic from ROS param server
    mobilenet_src_info = rospy.get_param("/wheelchair_robot/param/left_camera_info") #get camera info topic from ROS param server
    self.image_sub = message_filters.Subscriber(mobilenet_src, Image)
    self.info_sub = message_filters.Subscriber(mobilenet_src_info, CameraInfo)

    ts = message_filters.TimeSynchronizer([self.image_sub, self.info_sub], 10)
    ts.registerCallback(self.callback)

    self.pub_annotated_image = rospy.Publisher("/wheelchair_robot/mobilenet/annotated_image",Image, queue_size=10) #publish annotated image
    self.pub_annotated_image_info = rospy.Publisher("/wheelchair_robot/mobilenet/camera_info", CameraInfo, queue_size=1000)
    self.pub_detected_objects = rospy.Publisher("/wheelchair_robot/mobilenet/detected_objects", mobilenet, queue_size=1000)
    self.pub_raw_image = rospy.Publisher("/wheelchair_robot/mobilenet/raw_image", Image, queue_size=10) #publisher for unannotated images


  def callback(self, data, ros_cinfo):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      obj = String()
      addFrame()
      mobilenet_confidence_threshold = rospy.get_param("/wheelchair_robot/param/mobilenet/confidence_threshold")
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

    mobilenet_msg = mobilenet()

    #objectList = String()
    objectNoInFrame = 0
    for detection in output[0, 0, :, :]:
        confidence = detection[2]
        if confidence > mobilenet_confidence_threshold:
            #print("confidence threshold" + str(mobilenet_confidence_threshold))
            class_id = detection[1]
            class_name=id_class_name(class_id,classNames) #add +1 to class_id for fast-resnet
            if DEBUG_printObjectDetected:
              print(str(str(class_id) + " " + str(detection[2])  + " " + class_name))
            #add object logger here
            #objectLog.append(class_name)
            #objectConfLog.append(confidence)

            box_x = detection[3] * image_width
            box_y = detection[4] * image_height
            box_width = detection[5] * image_width
            box_height = detection[6] * image_height

            label = "{}: {:.2f}".format(class_name, confidence)
            cv2.rectangle(image, (int(box_x), int(box_y)), (int(box_width), int(box_height)), (23, 230, 210), 2)
            cv2.putText(image,label ,(int(box_x), int(box_y+.02*image_height)),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 250), 2)
            #class_name_array += class_name
            #frameCount = frameCount + 1
            #objectList.join(class_name)

            #mobilenet.objectName.append(class_name)
            #pub_detected_objects.objectName.append(class_name)
            if (class_name != "person") and (class_name != "dog") and (class_name != "cat"): #list of objects not to record in the environment
              mobilenet_msg.object_name.append(class_name)
              mobilenet_msg.object_confidence.append(confidence)
              mobilenet_msg.box_x.append(box_x)
              mobilenet_msg.box_y.append(box_y)
              mobilenet_msg.box_width.append(box_width)
              mobilenet_msg.box_height.append(box_height)
              objectNoInFrame += 1
              mobilenet_msg.totalObjectsInFrame = objectNoInFrame

            if DEBUG_noObjectsInFrame:
              print("total objects in frame are " , objectNoInFrame)
            
    #obj = objectList
    #cv2.imshow('image', image)




    #sleep(0.1)
    #cv2.waitKey(3)

    try:
      if (mobilenet_msg.totalObjectsInFrame != 0):
        mobilenet_msg.header.stamp = rospy.Time.now()
        mobilenet_msg.camera_timestamp = data.header.stamp
        self.pub_detected_objects.publish(mobilenet_msg)

        self.rosimgannotated = Image()
        self.rosimgannotated.header.stamp = rospy.Time.now()
        self.rosimgannotated = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.rosimgannotated.header.frame_id = "zed_left_camera_optical_frame"
        self.pub_annotated_image.publish(self.rosimgannotated) #publish annotated image
        self.pub_annotated_image_info.publish(ros_cinfo) #publish annotated image camera info
        self.rosimgraw = Image()
        self.rosimgraw.header.stamp = rospy.Time.now()
        self.rosimgraw = data
        self.rosimgraw.header.frame_id = "zed_left_camera_optical_frame"
        self.pub_raw_image.publish(self.rosimgraw)
      else:
        self.rosimgannotated = Image()
        self.rosimgannotated.header.stamp = rospy.Time.now()
        self.rosimgannotated = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.rosimgannotated.header.frame_id = "zed_left_camera_optical_frame"
        self.pub_annotated_image.publish(self.rosimgannotated) #publish annotated image
        self.pub_annotated_image_info.publish(ros_cinfo) #publish annotated image camera info
        self.rosimgraw = Image()
        self.rosimgraw.header.stamp = rospy.Time.now()
        self.rosimgraw = data
        self.rosimgraw.header.frame_id = "zed_left_camera_optical_frame"
        self.pub_raw_image.publish(self.rosimgraw)

    except CvBridgeError as e:
        print(e)

def foundObjectsFileWrite():
  #new section saves the varience of objects into a txt file "found-objects.txt"
    # record object type in a file - 1 file per training - overwrite when finished.
    roomNameParam = rospy.get_param("/wheelchair_param/user/room_name") #get room name from user
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
  rospy.init_node('mobilenet', anonymous=False)
  try:
    rospy.spin()
    print("Shutting down")
    #foundObjectsFileWrite()


  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
