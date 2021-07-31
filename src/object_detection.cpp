/*
 * object_detection.cpp
 * mobilenet
 * version: 0.1.0 Majestic Maidenhair
 * Status: pre-Alpha
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "ros/package.h" //find ROS packages, needs roslib dependency
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <sstream>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace std;
//using namespace cv;
//using namespace dnn;

const int DEBUG_doesPkgExist = 0;
const int DEBUG_populateClassNames = 0;
const int DEBUG_main = 0;

ros::NodeHandle *ptr_n;

std::string wheelchair_dump_loc;

std::vector<std::string> class_names;
int total_class_names = 0;

//function for printing space sizes
void printSeparator(int spaceSize) {
    if (spaceSize == 0) {
        printf("--------------------------------------------\n");
    }
    else {
        printf("\n");
        printf("--------------------------------------------\n");
        printf("\n");
    }
}

//does the wheelchair dump package exist in the workspace?
std::string doesPkgExist(std::string pkg_name) {
    std::string getPkgPath;
    if (ros::package::getPath(pkg_name) == "") {
        cout << "FATAL:  Couldn't find package " << pkg_name << "\n";
        cout << "FATAL:  Closing node. \n";
        if (DEBUG_doesPkgExist) {
            cout << getPkgPath << endl;
        }
        ros::shutdown();
        exit(0);
    }
    else {
        getPkgPath = ros::package::getPath(pkg_name);
        if (DEBUG_doesPkgExist) {
            cout << getPkgPath << endl;
        }
    }
    return getPkgPath;
}

void populateClassNames(std::string objects_list_loc) {
    ifstream FILE_READER(objects_list_loc); //open file
    int objectNumber = 0; //iterate on each object
    if (FILE_READER.peek() == std::ifstream::traits_type::eof()) {
        //don't do anything if next character in file is eof
        cout << "file is empty" << endl;
    }
    else {
        std::string line;
        while (getline(FILE_READER, line)) { //go through line by line
            //add to vector
            class_names.push_back(line);
            objectNumber++;
        }
        total_class_names = objectNumber;
    }
}

auto model = cv::dnn::readNetFromTensorflow(
"/home/tomos/ros/wheelchair/catkin_ws/src/mobilenet/scripts/models/frozen_inference_graph.pb", 
"/home/tomos/ros/wheelchair/catkin_ws/src/mobilenet/scripts/models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt");


int main(int argc, char **argv) {
    wheelchair_dump_loc = doesPkgExist("wheelchair_dump");//check to see if dump package exists
    std::string objects_list_loc = wheelchair_dump_loc + "/dump/object_detection/objects.txt"; //set path for dacop file (object info)
    populateClassNames(objects_list_loc);
    ros::init(argc, argv, "mobilenet_object_detection");
    ros::NodeHandle n;
    ptr_n = &n;
    ros::Rate rate(10.0);

    while(ros::ok()) {

        if (DEBUG_main) {
            cout << "spin \n";
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}