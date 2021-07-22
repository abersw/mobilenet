#include <ros/ros.h>
#include "mobilenet/Annotations.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

std::string requestObjectName;
static const std::string OPENCV_WINDOW = "Image window";

ros::NodeHandle *ptr_n;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/zed/zed_node/left/image_rect_color", 1, 
      &ImageConverter::imageCb, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    static int image_count = 0;                                // added this
    std::stringstream sstream;                               // added this
    sstream << "my_image" << image_count << ".png" ;                  // added this
    ROS_ASSERT( cv::imwrite( sstream.str(),  cv_ptr->image ) );      // added this
    image_count++;                                      // added this


    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

bool annotation_cb(mobilenet::Annotations::Request  &req,
         mobilenet::Annotations::Response &res)
{

requestObjectName = req.object; //get service request
std::cout << "called: " << requestObjectName << std::endl; //print service request
std::string pathLocation = "/home/tomos/ros/wheelchair/catkin_ws/"; //location of dir to save to
std::string annotationLocation = pathLocation + requestObjectName + ".jpg"; //append file name to path location
std::cout << annotationLocation << std::endl; //print out path location
ptr_n->setParam("/wheelchair_robot/image_saver_object_annotation/filename_format", annotationLocation); //set path location in parameter server
std::string s;
    if (ptr_n->getParam("/wheelchair_robot/image_saver_object_annotation/filename_format", s)) //get parameter to confirm
    {
      ROS_INFO("Got param: %s", s.c_str()); //print out parameter
    }
    else
    {
      ROS_ERROR("Failed to get param 'my_param'"); //couldn't retrieve parameter
    }

ros::ServiceClient client = ptr_n->serviceClient<std_srvs::Empty>("/wheelchair_robot/image_saver_object_annotation/save"); //call service with empty type
std_srvs::Empty srv;
if (client.call(srv))
{
  ROS_INFO("successfully called service"); //service successfully called
}
else {
  ROS_WARN("oops"); //service failed to call
}
//my_package::Foo foo;
ImageConverter ic;

//ros::Subscriber sub = ptr_n->subscribe("/wheelchair_robot/mobilenet/annotated_image", 1, imageCallback);
    
  /*res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);*/




  /*cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
    
    std::stringstream sstream;                               // added this
    sstream << "my_image" << image_count << ".png" ;                  // added this
    ROS_ASSERT( cv::imwrite( sstream.str(),  cv_ptr->image ) );      // added this*/

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mobilenet_Annotations_Service");
  ros::NodeHandle n;
  ptr_n = &n;
  

  ros::ServiceServer service = n.advertiseService("/wheelchair_robot/service/mobilenet/annotations", annotation_cb);
  ROS_INFO("Ready to save mobilenet annotations");
  ros::spin();

  return 0;
}