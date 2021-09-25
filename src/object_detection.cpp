/*
 * object_detection.cpp
 * mobilenet
 * version: 0.1.0 Majestic Maidenhair
 * Status: pre-Alpha
*/

#include "tof_tool/tof_tool_box.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>

using namespace std;
//using namespace cv;
//using namespace dnn;

static const int DEBUG_object_list_loc = 0;
static const int DEBUG_getImageSize = 0;

TofToolBox *tofToolBox;
image_transport::Publisher *pub_annotated_image;

cv::dnn::Net net;

std::string wheelchair_dump_loc;
std::vector<std::string> Names;
int total_class_names = 0;

int cvImageHeight = 0;
int cvImageWidth = 0;

static bool populateClassNames(std::string objects_list_loc) {
	// Open the File
	std::ifstream in(objects_list_loc.c_str());
	// Check if object is valid
	if(!in.is_open()) {
        return false;
    }
	std::string str;
	// Read the next line from File untill it reaches the end.
	while (std::getline(in, str)) {
		// Line contains string of length > 0 then save it in vector
		if(str.size()>0) {
            Names.push_back(str);
        }
	}
	// Close The File
	in.close();
	return true;
}

void getImageSize(cv::Mat cvImage) {
    cvImageHeight = cvImage.cols;
    cvImageWidth = cvImage.rows;
    if (DEBUG_getImageSize) {
        cout << "Image height: " << cvImageHeight << ", Image width: " << cvImageWidth << endl;
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& ros_msg_in) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(ros_msg_in, sensor_msgs::image_encodings::BGR8);
        cv::Mat cvImage = cv_ptr->image;
        getImageSize(cvImage);
        //getImageSize(cv_ptr->image);

        /*sensor_msgs::Image ros_msg_out;
        ros_msg_out = cv_ptr->toImageMsg();*/
        sensor_msgs::ImagePtr ros_msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvImage).toImageMsg();
        pub_annotated_image->publish(ros_msg_out);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv) {
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    wheelchair_dump_loc = tofToolBox->doesPkgExist("wheelchair_dump");//check to see if dump package exists
    std::string objects_list_loc = wheelchair_dump_loc + "/dump/object_detection/objects.txt"; //set path for dacop file (object info)
    if (DEBUG_object_list_loc) {
        cout << "location of objects: " << objects_list_loc << endl;
    }

    net = cv::dnn::readNetFromTensorflow(
    "/home/tomos/ros/wheelchair/catkin_ws/src/mobilenet/scripts/models/frozen_inference_graph.pb", 
    "/home/tomos/ros/wheelchair/catkin_ws/src/mobilenet/scripts/models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt");

    if (net.empty()){
        cout << "init the model net error";
        exit(-1);
    }

    bool result = populateClassNames(objects_list_loc);
	if(!result) {
        cout << "loading labels failed";
        exit(-1);
	}

    ros::init(argc, argv, "mobilenet_object_detection");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
//    image_transport::Subscriber sub = it.subscribe("/zed/zed_node/left/image_rect_color", 1, imageCallback);
    image_transport::Subscriber sub = it.subscribe("/kitchen_image_publisher/image_raw", 1, imageCallback);
    image_transport::Publisher pub_annotated_image_local = it.advertise("/wheelchair_robot/mobilenet/image_raw", 1);
    pub_annotated_image = &pub_annotated_image_local;
    ros::Rate rate(10.0);
    while (ros::ok()) {
        ros::spin();
        rate.sleep();
    }
    return 0;
}