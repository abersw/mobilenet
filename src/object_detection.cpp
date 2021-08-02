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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
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

static const std::string OPENCV_WINDOW = "Image window";

const int DEBUG_doesPkgExist = 0;
const int DEBUG_populateClassNames = 0;
const int DEBUG_main = 0;

ros::NodeHandle *ptr_n;

std::string camera_src;

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
        cout << "class name list is empty, closing node" << endl;
		exit(0); //close down node
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

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    public:
    ImageConverter()
    : it_(nh_) {
        nh_.getParam("/wheelchair_robot/param/left_camera", camera_src);
        image_sub_ = it_.subscribe("/kitchen_image_publisher/image_raw", 1, &ImageConverter::imageCb, this); //subscriber
        image_pub_ = it_.advertise("/wheelchair_robot/mobilenet/annotated_image", 1);
        cv::namedWindow(OPENCV_WINDOW);
    }
    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImage img_ptr; //pointer for cv_img to ros_img
            sensor_msgs::Image img_msg; //create output image message
            cv::Mat cvImage = cv_ptr->image;
            int cvImageHeight = cvImage.cols;
            int cvImageWidth = cvImage.rows;

            cv::Mat blob = cv::dnn::blobFromImage(cvImage, 1.0, cv::Size(300, 300), cv::Scalar(127.5, 127.5, 127.5), true, false);
            model.setInput(blob); //create blob from image
            cv::Mat outputImage = model.forward();
            cv::Mat detectionMat(outputImage.size[2], outputImage.size[3], CV_32F, outputImage.ptr<float>());

            for (int i = 0; i < detectionMat.rows; i++){
                int class_id = detectionMat.at<float>(i, 1);
                float confidence = detectionMat.at<float>(i, 2);
                if (confidence > 0.5){
                    int box_x = static_cast<int>(detectionMat.at<float>(i, 3) * cvImage.cols);
                    int box_y = static_cast<int>(detectionMat.at<float>(i, 4) * cvImage.rows);
                    int box_width = static_cast<int>(detectionMat.at<float>(i, 5) * cvImage.cols - box_x);
                    int box_height = static_cast<int>(detectionMat.at<float>(i, 6) * cvImage.rows - box_y);
                    cv::rectangle(cvImage, cv::Point(box_x, box_y), cv::Point(box_x+box_width, box_y+box_height), cv::Scalar(255,255,255), 2);
                    cv::putText(cvImage, class_names[class_id-1].c_str(), cv::Point(box_x, box_y-5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1);
                    cv::imshow("image", cvImage);
                    //image_pub_.publish(cvImage->toImageMsg());
                    cv::waitKey(0);
                    cv::destroyAllWindows();
                }
            }
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
};

int main(int argc, char **argv) {
    wheelchair_dump_loc = doesPkgExist("wheelchair_dump");//check to see if dump package exists
    std::string objects_list_loc = wheelchair_dump_loc + "/dump/object_detection/objects.txt"; //set path for dacop file (object info)
    populateClassNames(objects_list_loc);
    ros::init(argc, argv, "mobilenet_object_detection");

    ImageConverter ic;
    ros::spin();

    return 0;
}