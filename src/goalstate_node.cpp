// ros includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include "std_msgs/String.h"

// robot config
#include "robot_config.h"

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

/***************************
* LOCAL DEFINITIONS
***************************/
// subscribe to the raw camera image
image_transport::Subscriber image_sub;

// label of the GUI window showing the raw image of NAO's camera
static const char cam_window[] = "NAO Camera (raw image)";


/***************************
* CALLBACK - FUNCTIONS
***************************/
// callback function for vision
void visionCB(const sensor_msgs::ImageConstPtr& msg)
{
  // pointer on OpenCV image
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    // transform ROS image into OpenCV image
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)		// throw an error msg. if conversion fails
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  imshow(cam_window, cv_ptr->image);

  waitKey(100);
}


/***************************
* MAIN
***************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "goalstate_node");
  ros::NodeHandle gs_node_nh;

  // using image_transport to publish and subscribe to images
  image_transport::ImageTransport image_tran(gs_node_nh);

  // subscriber for the top camera
  image_sub = image_tran.subscribe("nao_robot/camera/top/camera/image_raw", 1, &visionCB);

  // create a GUI window for the raw camera image
  namedWindow(cam_window, WINDOW_AUTOSIZE);

  ros::spin();

  return 0;
}
