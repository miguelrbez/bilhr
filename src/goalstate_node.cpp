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

//aruco markers
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <stdlib.h>

#include <string>

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

// Marker Detection
MarkerDetector MDetector;
CameraParameters cameraParameters;    // instance of class CameraParameters

// publisher for goalkeeper
ros::Publisher gs_pub;

// label of the GUI window showing the raw image of NAO's camera
static const char cam_window[] = "NAO Camera (raw image)";

// goalkeeper states
string state1 = "state1";
string state2 = "state2";
string state3 = "state3";
string state4 = "state4";
string state5 = "state5";

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

  // MARKER DETECTION
  Mat imgIn;
  img->image.copyTo(imgIn);
  std::vector<Marker> Markers;
  MDetector.detect(imgIn,Markers,cameraParameters,0.04);
  // For each marker, draw info and its boundaries in the image
  for (unsigned int i=0;i<Markers.size();i++) {
    cout<<Markers[i]<<endl;
    Markers[i].draw(imgIn,cv::Scalar(0,0,255),2);
  }
  imshow("Marker Detection", imgIn);

  // publish goalkeeper to rl_node
  std_msgs::String gs_msg;
  // setting msg
  gs_msg.data = state1;
  gs_pub.publish(gs_msg);

  waitKey(100);
}


/***************************
* MAIN
***************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "goalstate_node");
  ros::NodeHandle gs_node_nh;


  top_camera.distortion = Mat(1, 5, CV_32FC1);
  top_camera.distortion.at<float>(0,0) = -0.066494;
  top_camera.distortion.at<float>(0,1) =  0.095481;
  top_camera.distortion.at<float>(0,2) = -0.000279;
  top_camera.distortion.at<float>(0,3) =  0.002292;
  top_camera.distortion.at<float>(0,4) =  0;

  top_camera.camera = Mat(3, 3, CV_32FC1, 0.0);
  top_camera.camera.at<float>(0,0) = 551.543059;
  top_camera.camera.at<float>(1,1) = 553.736023;
  top_camera.camera.at<float>(2,2) = 1.0;
  top_camera.camera.at<float>(0,2) = 327.382898;
  top_camera.camera.at<float>(1,2) = 225.026380;

  top_camera.focalLength = 581.25f;

  top_camera.horizontalFOV = 0.83147486; // 47.64 degrees in radians
  top_camera.verticalFOV   = 1.0641272;  // 60.97 degrees in radians

  cameraParameters.setParams(top_camera.camera, top_camera.distortion, cv::Size(640,480));
  cameraParameters.resize(cv::Size(640,480));

  // using image_transport to publish and subscribe to images
  image_transport::ImageTransport image_tran(gs_node_nh);

  // subscriber for the top camera
  image_sub = image_tran.subscribe("nao_robot/camera/top/camera/image_raw", 1, &visionCB);

  // publisher for goalkeeper state
  gs_pub = gs_node_nh.advertise<std_msgs::String>("goalkeeper", 10);

  // create a GUI window for the raw camera image
  namedWindow(cam_window, WINDOW_AUTOSIZE);

  ros::spin();

  return 0;
}
