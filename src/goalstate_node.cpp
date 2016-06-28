/*
 * goalstate_node is made by Emilia Skurzynska
 */

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

// ArUco markers
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <tf/LinearMath/Vector3.h>

#include <string>

// robot config
//#include "robot_config.h"

using namespace std;
using namespace cv;
using namespace aruco;

struct camera_params {
    Mat distortion;
    Mat camera;
    float focalLength;
    float horizontalFOV;
    float verticalFOV;
};

camera_params top_camera;

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
int no_states = 5;
string state1 = "state1";
string state2 = "state2";
string state3 = "state3";
string state4 = "state4";
string state5 = "state5";

// Show all windows
bool show_result = true;
bool show = false;

// MARKERS IDS: 1, 2

// HELPER FUNCTIONS

/**
* Function rotating the image.
* @param img            image that will be rotated, Mat type,
* @param center         center of the rotation, Point2f type.
*/
Mat rotate(Mat img,Point2f center){
  Mat rot = getRotationMatrix2D(center, 43, 1);
  Mat result;
  warpAffine(img, result, rot, img.size());
  if (show)
    imshow("Rotation", result);
  return result;
}

/**
* CLAHE algorithm.
* @param img            image which histogram should be normalized, Mat type.
* @param ClipLimit      algorithm parameter on which the "grain" depends, float type.
* @param TilesGridSize  algorithm parameter on which the size of the tiles depends, float type.
*/
Mat Clahe(Mat img, float ClipLimit, int TilesGridSize){
    // normalize image contrast & luminance
    Mat norm_img;
    cvtColor(img, norm_img, CV_BGR2Lab);

    // Extract the L channel
    std::vector<Mat> lab_planes(3);
    split(norm_img, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(ClipLimit);
    clahe->setTilesGridSize(Size(TilesGridSize,TilesGridSize));
    Mat dst;
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    merge(lab_planes, norm_img);
    cvtColor(norm_img, norm_img, CV_Lab2BGR);
    if (show)
      imshow("CLAHE", norm_img);
    return norm_img;
}

/**
* Simple interpolation based on two points algorithm.
* @param img            image which histogram should be normalized, Mat type.
* @param ClipLimit      algorithm parameter on which the "grain" depends, float type.
* @param TilesGridSize  algorithm parameter on which the size of the tiles depends, float type.
*/
vector<float> simple_interpolation(Mat img, Point2f first_point, Point2f second_point){
    float a,b;
    a = (first_point.y - second_point.y) / (first_point.x - second_point.x);
    b = first_point.y - a * first_point.x;

    float difference = abs(first_point.x - second_point.x) / (2);
    Mat show = img.clone();

  	Point2f point;
  	point.y = (first_point.y - second_point.y) / 2;
  	point.x = (first_point.x - second_point.x) / 2;

    vector<float> parameters = {a, b};
    return parameters;
}

/**
 * Compares contours regarding area sizes
 * @param contour_1 first compared contour, vector<cv::Point> type,
 * @param contour_2 second compared contour, vector<cv::Point> type.
 */
bool compare_area(vector<cv::Point> contour_1,vector<cv::Point> contour_2){
    // rotated rectangle
    RotatedRect boundRect1 = minAreaRect(Mat(contour_1));
    RotatedRect boundRect2 = minAreaRect(Mat(contour_2));
    // calculate area
    float area_1 = boundRect1.size.width*boundRect1.size.height;
    float area_2 = boundRect2.size.width*boundRect2.size.height;

    return area_1 < area_2;
}

/**
 * Compares contours regarding sizes
 * @param contour_1 first compared contour, vector<cv::Point> type,
 * @param contour_2 second compared contour, vector<cv::Point> type.
 */
bool compare_size(vector<cv::Point> contour_1,vector<cv::Point> contour_2){
  return contour_1.size() < contour_2.size();
}

/**
 * Compares contours regarding y coordinates of the center
 * @param contour_1 first compared contour, vector<cv::Point> type,
 * @param contour_2 second compared contour, vector<cv::Point> type.
 */
bool compare_y(vector<cv::Point> contour_1,vector<cv::Point> contour_2){
    // rotated rectangle
    RotatedRect boundRect1 = minAreaRect(Mat(contour_1));
    RotatedRect boundRect2 = minAreaRect(Mat(contour_2));
    // calculate area
    float point_1 = boundRect1.center.y;
    float point_2 = boundRect2.center.y;

    return point_1 < point_2;
}

/**
 * Compares contours regarding y coordinates of the center
 * @param contour_1 first compared contour, vector<cv::Point> type,
 * @param contour_2 second compared contour, vector<cv::Point> type.
 */
bool compare_x(vector<cv::Point> contour_1,vector<cv::Point> contour_2){
    // rotated rectangle
    RotatedRect boundRect1 = minAreaRect(Mat(contour_1));
    RotatedRect boundRect2 = minAreaRect(Mat(contour_2));
    // calculate area
    float point_1 = boundRect1.center.x;
    float point_2 = boundRect2.center.x;

    return point_1 < point_2;
}

/**
 * Extracts orange color
 * @param src   image from which the orange color should be extracted, Mat type.
 */
Mat extractColor(Mat src)
{
  Scalar orange_min = Scalar(  0, 150, 150);
  Scalar orange_max = Scalar(  20, 255, 255);

  Mat norm_img = Clahe(src,4,12);
  Mat hsv_image, top_image;

  // convert image to HSV
  cvtColor( norm_img, hsv_image, CV_BGR2HSV);
  Mat blurred;
  // Filter image without losing borders
  bilateralFilter ( hsv_image, blurred, 5, 10, 2 );

  cv::inRange(blurred, orange_min, orange_max, top_image);

  Mat erosion;
  Mat dilate;
  Mat element;

  cv::dilate(top_image,dilate,element); //Dilate the image
  cv::erode(dilate,erosion,element); // Erode the image

  if (show)
    imshow("Color extracted", erosion);
  return erosion;
}

/**
 * Extract rotated rectangles bounding the contours found in the image
 * @param img   image from which the parts should be extracted, Mat type.
 */
vector<RotatedRect> extractAllParts(Mat img, vector <Point2f> centers)
{
  // Bounding rectangles for each contour
  vector<RotatedRect> boundRect;

  // Detecting Contours
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;

  /// Find contours
  cv::findContours(img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

  // Get rid of the contours outside the polls
  RotatedRect rect;
  for (auto it = contours.begin(); it != contours.end(); )
  {
      rect = minAreaRect(Mat(*it));
      if (rect.center.x < centers[0].x || rect.center.x > centers[1].x)
      {
          it = contours.erase(it);
      }
      else
      {
          ++it;
      }
  }

  if (!(contours.size() > 0)){
      cout<<("No contours found");
      return boundRect;
  }
  else{
    //Save the blobs - probably the goal keeper
    int ind, area_size,min_area = 20;
    for (int i = 0; i < contours.size(); i++) {
        // contour
        boundRect.push_back(minAreaRect(Mat(contours[i])));
        area_size = boundRect[i].size.width*boundRect[i].size.height;
        if (area_size > min_area) {
            // rotated rectangle
            Point2f rect_points[4];
            boundRect[i].points(rect_points);
        }
      }
  }
  //display(img,boundRect);
  return boundRect;
}

/**
 * Extract the center of the goal keeper
 * @param img   image from which the parts should be extracted, Mat type.
 */
Point2f extractGoalKeeper(Mat img, vector <Point2f> centers)
{
  Point2f goal_keeper;
  int sort_type = 3;
  int part_number = 0;

  // Bounding rectangles for each contour
  vector<RotatedRect> boundRect;
  // Detecting Contours
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;

  /// Find contours
  cv::findContours(img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  if (!(contours.size() > 0)){
      cout<<("No contours found");
      return goal_keeper;
  }
  else{

  if(sort_type == 1){
    sort(contours.begin(),contours.end(),compare_y);
    part_number = 2;
  }
  else if(sort_type == 2){
    sort(contours.begin(),contours.end(),compare_x);
    part_number = 2;
  }
  else if(sort_type == 3){
    sort(contours.begin(),contours.end(),compare_area);
    part_number = 1;
  }
  else {
    sort(contours.begin(),contours.end(),compare_size);
    part_number = 1;
  }

  // Get rid of the contours outside the polls
  RotatedRect rect;
  for (auto it = contours.begin(); it != contours.end(); )
  {
      rect = minAreaRect(Mat(*it));
      if (rect.center.x < centers[0].x || rect.center.x > centers[1].x)
      {
          it = contours.erase(it);
      }
      else
      {
          ++it;
      }
  }

  reverse(contours.begin(),contours.end());
  vector<vector<Point> > contours_poly( part_number );

  //Save biggest blobs - probably the goal keeper
  int ind, area_size,min_area = 20;
  for (int i = 0; i < part_number; i++) {
    if (i >= contours.size()){
      continue;
    }
    else{
      // contour
      boundRect.push_back(minAreaRect(Mat(contours[i])));
      area_size = boundRect[i].size.width*boundRect[i].size.height;
      if (area_size > min_area) {
          // rotated rectangle
          Point2f rect_points[4];
          boundRect[i].points(rect_points);
      }
    }
  }
  if(sort_type == 1){
    goal_keeper.x = abs(boundRect[0].center.x - boundRect[1].center.x)/2;
    goal_keeper.y = abs(boundRect[0].center.y - boundRect[1].center.y)/2;
  }
  else if(sort_type == 2){
    goal_keeper.x = abs(boundRect[0].center.x - boundRect[1].center.x)/2;
    goal_keeper.y = abs(boundRect[0].center.y - boundRect[1].center.y)/2;
  }
  else if(sort_type == 3){
    goal_keeper = boundRect[0].center;
  }
  else {
    goal_keeper = boundRect[0].center;
  }
  //display(img,boundRect);
  return goal_keeper;
}
}

/**
 * Display boxes on the given image
 * @param show      image on which the boxes will be shown, Mat type,
 * @param boundRect vector storing information about the boxes.
 */
void displayBoxes(Mat img, vector<RotatedRect> boundRect)
{
  Mat shows = img.clone();
  int ind, area_size, min_area = 20;
  for (int i = 0; i < boundRect.size(); i++) {
    area_size = boundRect[i].size.width * boundRect[i].size.height;
    if (area_size > min_area) {
        // rotated rectangle
        Point2f rect_points[4];
        boundRect[i].points(rect_points);
        for (int j = 0; j < 4; j++)
          line(shows, rect_points[j], rect_points[(j + 1) % 4], cv::Scalar(255, 0, 0), 1, 8);
        cv::circle(shows, boundRect[i].center, 5, cv::Scalar(0, 0, 0));

    }
  }
  if (boundRect.size()>1){
    line(shows,boundRect[0].center,boundRect[1].center,1,8,0);
    simple_interpolation(shows,boundRect[0].center,boundRect[1].center);
  }
  else if (boundRect.size()==0){
    cv::circle(shows, boundRect[0].center, 5, cv::Scalar(0, 0, 255));
  }
  if (show)
    imshow("Bounding boxes", shows);
}

/**
 * Extract positions of the markers
 * @param img   image from which the markers should be extracted, Mat type.
 */
vector<Point2f> extractMarkers(Mat img)
{
  // MARKER DETECTION
  std::vector<Marker> Markers;
  std::vector<Point2f> centers;
  float markerSize = (float)0.1;
  Point2f left, right;
  Mat im_show = img.clone();

  // Detection Parameters
  MDetector.setWarpSize(100);
  MDetector.enableLockedCornersMethod(true);
  MDetector.setMinMaxSize(0.01, 0.5);

  // Detect the markers
  MDetector.detect(im_show,Markers,cameraParameters,0.04);
  // For each marker, draw info and its boundaries in the image
  if (Markers.size()>1){
    for (unsigned int i=0;i<Markers.size();i++) {
      Markers[i].calculateExtrinsics(markerSize, cameraParameters, false);
      Markers[i].draw(im_show, cv::Scalar(0,0,255),2);
      // Check the position of the markers based on their ids
      if(Markers[i].id == 1){
        left.x = abs(Markers[i][0].x + Markers[i][2].x)/2;
        left.y = abs(Markers[i][0].y + Markers[i][2].y)/2;
      }
      if(Markers[i].id == 2){
        right.x = abs(Markers[i][0].x + Markers[i][2].x)/2;
        right.y = abs(Markers[i][0].y + Markers[i][2].y)/2;
      }
    }
    // Draw circles on the image
    cv::circle(im_show, left, 10, cv::Scalar(0, 0, 255));
    cv::circle(im_show, right, 10, cv::Scalar(0, 0, 255));
    // Draw circles on the image
    centers.push_back(left);
    centers.push_back(right);
    if (show){
      imshow("Marker Detection", im_show);
    }
  }
  else{
    //ROS_INFO("ERROR, MARKERS NOT FOUND");
  }
  return centers;
}

int getTheState(Mat img, vector<Point2f> centers, Point2f goal_keeper)
{
  Mat im_show = img.clone();
  // Check the distance of the markers
  float distance_polls = sqrt((centers[0].x - centers[1].x)*(centers[0].x - centers[1].x) + (centers[0].y - centers[1].y)*(centers[0].y - centers[1].y));
  // Check the interpolation of the points
  vector<float> parameters = simple_interpolation(im_show, centers[0], centers[1]);

  // The point of the crossing of the perpendicular line to the line connecting the points
  Point2f cross;
  // Perpendicular line
  float a,b;
  a = -1/parameters[0];
  b = goal_keeper.y - goal_keeper.x * a;
  // Cross point
  cross.x =  (parameters[1] - b) / (a - parameters[0]);
  cross.y =  a * cross.x + b;
  // The distance between the poll and the goal_keeper
  float distance_goal = sqrt((centers[0].x - cross.x)*(centers[0].x - cross.x) + (centers[0].y - cross.y)*(centers[0].y - cross.y));

  // Getting the state
  int state = floor(no_states * distance_goal/distance_polls);

  if (show_result){
    line(im_show, cross, goal_keeper, cv::Scalar(255, 0, 0), 1, 8);
    line(im_show, centers[0], goal_keeper, cv::Scalar(255, 255, 255), 4, 8);
    line(im_show, centers[0], centers[1], cv::Scalar(0, 255, 0), 1, 8);
    imshow("Distances", im_show);
  }
  ROS_INFO("Current state: %d", state);
  return state;
}

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

  // Rotate the image
  Mat result, rotated;
  Point2f center(320.0,240.0);
  rotated = rotate(cv_ptr->image,center);

  // Extract the markers - the polls
  vector<Point2f> centers = extractMarkers(rotated);
  if (centers.size() > 0){
    // Extract the color, the goal keeper
    result = extractColor(rotated);
    vector<RotatedRect> rectangles = extractAllParts(result,centers);
    if (rectangles.size() > 0){
      if (show)
        displayBoxes(rotated,rectangles);
      Point2f goal_keeper = extractGoalKeeper(result,centers);
      int state = getTheState(rotated,centers,goal_keeper);
      // publish goalkeeper to rl_node
      std_msgs::Int32 gs_msg;
      // setting msg
      gs_msg.data = state;
      gs_pub.publish(gs_msg);
    }
  }
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
  gs_pub = gs_node_nh.advertise<std_msgs::Int32>("goalkeeper", 10);

  // create a GUI window for the raw camera image
  namedWindow(cam_window, WINDOW_AUTOSIZE);

  ros::spin();

  return 0;
}
