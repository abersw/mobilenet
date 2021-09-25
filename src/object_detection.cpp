/*
 * object_detection.cpp
 * mobilenet
 * version: 0.1.0 Majestic Maidenhair
 * Status: pre-Alpha
*/

#include "tof_tool/tof_tool_box.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
//using namespace cv;
//using namespace dnn;

static const int DEBUG_object_list_loc = 0;

TofToolBox *tofToolBox;

std::string wheelchair_dump_loc;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    //do stuff
}

int main(int argc, char **argv) {
    TofToolBox tofToolBox_local;
    tofToolBox = &tofToolBox_local;

    wheelchair_dump_loc = tofToolBox->doesPkgExist("wheelchair_dump");//check to see if dump package exists
    std::string objects_list_loc = wheelchair_dump_loc + "/dump/object_detection/objects.txt"; //set path for dacop file (object info)
    if (DEBUG_object_list_loc) {
        cout << "location of objects: " << objects_list_loc << endl;
    }

    ros::init(argc, argv, "mobilenet_object_detection");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
//    image_transport::Subscriber sub = it.subscribe("/zed/zed_node/left/image_rect_color", 1, imageCallback);
    image_transport::Subscriber sub = it.subscribe("/kitchen_image_publisher/image_raw", 1, imageCallback);
    image_transport::Publisher pub_annotated_image_local = it.advertise("/wheelchair_robot/mobilenet/annotated_image", 1);
    ros::Rate rate(10.0);
    while (ros::ok()) {
        ros::spin();
        rate.sleep();
    }
    return 0;
}