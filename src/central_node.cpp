/*
 *   Author:      Erhard Wieser
 *
     Description: A node for sending and receiving sensorimotor data from the NAO robot.

     IP of our robot: 10.152.246.242
     start ROS with
      roslaunch nao_bringup nao_full_py.launch nao_ip:=10.152.246.242 roscore_ip:=10.152.246.128
      roslaunch nao_apps tacticle.launch nao_ip:=10.152.246.242 roscore_ip:=10.152.246.128
      roscore_ip: our terminal IP
      nao_ip: NAO IP
     Group E
        Adam Zylka
        Miguel Rodriguez
        Martin Patz
 */


#include <ros/ros.h>

// ROS and OpenCV image processing
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

// own files
#include "robot_config.h"
#include "mlp.h"

// saving values into file
#include <fstream>
#include <iostream>


using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

namespace enc = sensor_msgs::image_encodings;


// subscribers to tactile and touch sensors
ros::Subscriber tactile_sub;
ros::Subscriber bumper_sub;

// publisher to robot joints for sending the target joint angles
ros::Publisher target_joint_state_pub;

// joint stiffnesses
ros::Publisher stiffness_pub;

// waving
ros::Publisher waving_pub;

// received motor state of the HEAD
double motor_head_in[HEAD_DOF];

// received motor state of the LEFT ARM
double motor_l_arm_in[L_ARM_DOF];

// received motor state of the RIGHT ARM
double motor_r_arm_in[R_ARM_DOF];

// label of the GUI window showing the raw image of NAO's camera
static const char cam_window[] = "NAO Camera (raw image)";

// set positions for both arms
robot_specific_msgs::JointAnglesWithSpeed L_ARM_HomePosition;

// fileName
char dataName[] = "data.txt";
vector<vector <double> > inputData;
vector<vector <double> > outputData;
int samples;

// samples
std::vector< std::vector<double> > input_samples;
std::vector< std::vector<double> > output_samples;

// normalized position of object in picture and of shoulder joints
float normX;
float normY;
float normShoulder1;
float normShoulder2;

// output position of shoulder joints
double reachShoulder1;
double reachShoulder2;

// Flag true to add sample directly (not loading file)
bool add_sample_flag = false;

// MLP of robot
MLP mlp(2, 8, 2, 0.005, 0.005, 0.00003);
mlp.set_max_iterations(200000);


// set the stiffness
void setStiffness(float value)
{
  robot_specific_msgs::JointState target_joint_stiffness;

  // set stiffnesses of HEAD joints
  target_joint_stiffness.name.clear();
  target_joint_stiffness.name.push_back("Head");
  target_joint_stiffness.effort.clear();
  for (int i=0; i<HEAD_DOF; i++)
      target_joint_stiffness.effort.push_back(value);

  stiffness_pub.publish(target_joint_stiffness);
}

void setStiffness(float value, std::string name)
{
  cout << "setting stiffnesses for " << name << " to " << value << endl;
  robot_specific_msgs::JointState target_joint_stiffness;
  target_joint_stiffness.name.clear();
  target_joint_stiffness.name.push_back(name);
  target_joint_stiffness.effort.clear();
  int dof;
  if (name == "Head")
    dof = HEAD_DOF;
  else if (name == "LArm")
    dof = L_ARM_DOF;
  else if (name == "RArm")
    dof = R_ARM_DOF;
  // TODO: put the other IDs

  for (int i = 0; i < dof; i++)
    target_joint_stiffness.effort.push_back(value);

  stiffness_pub.publish(target_joint_stiffness);
}

// sets stiffness for every joint independently
void setStiffness(float value[], std::string name)
{
  cout << "setting stiffnesses for " << name << " to " << endl;

  robot_specific_msgs::JointState target_joint_stiffness;
  target_joint_stiffness.name.clear();
  target_joint_stiffness.name.push_back(name);
  target_joint_stiffness.effort.clear();
  int dof;
  if (name == "Head")
    dof = HEAD_DOF;
  else if (name == "LArm")
    dof = L_ARM_DOF;
  else if (name == "RArm")
    dof = R_ARM_DOF;
  // TODO: put the other IDs

  for (int i = 0; i < dof; i++)
  {
    cout << value[i] << endl;

    target_joint_stiffness.effort.push_back(value[i]);
  }

  stiffness_pub.publish(target_joint_stiffness);
}


// initial values for bars to adjust color
const int MAX_VAL = 255;
int lowH = 0;
int highH = 76;
int lowS = 122;
int highS = 230;
int lowV = 179;
int highV = 255;

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

    Mat imgHSV, imgThreshold;
    // transform image to HSV
    cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV);

    // create trackbars
    namedWindow("control", CV_WINDOW_AUTOSIZE);
    cvCreateTrackbar("lowH", "control", &lowH, MAX_VAL);
    cvCreateTrackbar("highH", "control", &highH, MAX_VAL);
    cvCreateTrackbar("lowS", "control", &lowS, MAX_VAL);
    cvCreateTrackbar("highS", "control", &highS, MAX_VAL);
    cvCreateTrackbar("lowV", "control", &lowV, MAX_VAL);
    cvCreateTrackbar("highV", "control", &highV, MAX_VAL);

    // get hsv threshold
    inRange(imgHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), imgThreshold);

    //make hsv better
    erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    imshow("imgThreshold", imgThreshold);

    // detect edges
    Mat canny_output;
    int thresh = 100;
    Canny( imgThreshold, canny_output, thresh, thresh*2, 3 );
    //imshow("canny_output", canny_output);

    // get contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    // make rectangles and circles around all contours
    int contourSize = contours.size();

    // check if a object is found
    if(contourSize > 0)
    {
      vector<vector<Point> > contours_poly( contourSize );
      vector<Rect> boundRect( contourSize );
      vector<Point2f>center( contourSize );
      vector<float>radius( contourSize );
      for( int i = 0; i < contourSize; i++ )
      {
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
      }

      // search for biggest rectangle
      int biggestRectArea = 0, item = 0;
      for( int i = 0; i < contourSize; i++ )
      {
        if(biggestRectArea < boundRect[i].width*boundRect[i].height)
        {
          biggestRectArea = boundRect[i].width*boundRect[i].height;
          item = i;
        }
      }

      // draw circle around center
      int circleRadius = 3;
      Scalar circleColor(0, 0, 255);
      circle(cv_ptr->image, center.at(item), circleRadius, circleColor, 5);

      // terminal output for biggest blobb
      // cout << "pos " << center.at(item) << endl;

      // normalized position
      normX = center.at(item).x / cv_ptr->image.size().width;
      normY = center.at(item).y / cv_ptr->image.size().height;

      // terminal output normalized
      // cout << "normalized object position: " << normX << "\t" << normY << endl;
    }
    // when objects are not in the picture
    else
    {
      normX = 10;
      normY = 10;
    }

    // show the raw camera image
    imshow(cam_window, cv_ptr->image);

    waitKey(100);
}

// function for normalizing joint states of the shoulder
void normalizeShoulder()
{
  // shift position in order to calculate only positive joint pos
  float shoulder1 = motor_l_arm_in[0] + L_SHOULDER_PITCH_NEG_VALUE;
  float shoulder2 = motor_l_arm_in[1] + L_SHOULDER_ROLL_NEG_VALUE;

  // normalize
  normShoulder1 = shoulder1 / L_SHOULDER_PITCH_RANGE;
  normShoulder2 = shoulder2 / L_SHOULDER_ROLL_RANGE;
}

// function for de-normalizing output of mlp
void denormalizeOutput(std::vector<double> output)
{
  // de-normalize
  double denormShoulder1 = output[0] * L_SHOULDER_PITCH_RANGE;
  double denormShoulder2 = output[1] * L_SHOULDER_ROLL_RANGE;

  // shift position in order to calculate right joint pos
  reachShoulder1 = denormShoulder1 - L_SHOULDER_PITCH_NEG_VALUE;
  reachShoulder2 = denormShoulder2 - L_SHOULDER_ROLL_NEG_VALUE;
}

void loadData()
{
  // load data from file into array
  cout << "loading data for training... \n";
  ifstream dataFile(dataName);
  while(!dataFile.eof())
  {
    vector<double> input;
    vector<double> output;
    double tmp;

    // update samples
    samples++;

    // load pic x
    dataFile >> tmp;
    input.push_back(tmp);
    // load pic y
    dataFile >> tmp;
    input.push_back(tmp);

    // load input sample into vector
    inputData.push_back(input);

    // load first shoulder joint
    dataFile >> tmp;
    output.push_back(tmp);
    // load second shouder joint
    dataFile >> tmp;
    output.push_back(tmp);

    // load output sample into vector
    outputData.push_back(output);

    // add loaded sample data to mlp
    mlp.add_sample(inputData.back(), outputData.back());
  }
  cout << "loading data done\n";
}

//function to reach for the object with LArm
void reach ()
{
  //Input and output vectors
  std::vector<double> input;
  std::vector<double> output;

  //Load current position into Input vector
  input.push_back(normX);
  input.push_back(normY);

  // calculate normalized positions of shoulder
  output = mlp.calc_fwd_propagation(input);

  //de-normalize position of shoulder
  denormalizeOutput(output);

  // print inputs and outputs
  cout << normX << "\t" << normY << "\t" << reachShoulder1 << "\t" << reachShoulder2 << endl;  

}

// callback function for tactile buttons (TBs) on the head
void tactileCB(const robot_specific_msgs::TactileTouch::ConstPtr& __tactile_touch)
{
    // check TB 3 (rear)
    if (((int)__tactile_touch->button == 3) && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;
        setStiffness(0.8, "RArm");

        // tutorial 2
        // publish message to waving_node
        // std_msgs::String wave_msg;
        // wave_msg.data = 'b';
        // waving_pub.publish(wave_msg);
    }

    // check TB 2 (middle)
    if (((int)__tactile_touch->button == 2) && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;

        // tutorial 2
        // publish message to waving_node
        // std_msgs::String wave_msg;
        // wave_msg.data = 'm';
        // waving_pub.publish(wave_msg);

        // tutorial 3
        // load samples from file for training (if add_sample_flag is false)
        if (!add_sample_flag)
        {
          loadData();
        }

        // training mlp of robot
        cout << "Start training with " << mlp.nr_samples_ << " samples..." << endl;
        mlp.train();
        cout << "done" << endl;

    }

    // check TB 1 (front)
    if (((int)__tactile_touch->button == 1) && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;

        // tutorial 2
        // move left arms to home pos
        // setStiffness(0.8, "LArm");
        // target_joint_state_pub.publish(L_ARM_HomePosition);

        // tutorial 3
        // normalize shoulder
        normalizeShoulder();

        // save values into file
        cout << "writing values into file [posPic, jointPos]...";
        ofstream dataFile;
        dataFile.open(dataName, ios::app);
        dataFile << normX << "\t" << normY << "\t" << normShoulder1 << "\t" << normShoulder2 << "\n";
        dataFile.close();
        cout << "done " << endl;

        // add sample directly for training (not loading file) if add_sample_flag is true
        if (add_sample_flag)
        {
          cout << "Adding sample nr " << mlp.nr_samples_ << endl;
          cout << normX << "\t" << normY << "\t" << normShoulder1 << "\t" << normShoulder2 << endl
          input_samples.push_back({normX, normY});
          output_samples.push_back({normShoulder1, normShoulder2});
          mlp.add_sample(input_samples.back(), output_samples.back());
          cout << "done " << endl;
        }

    }

}


// callback function for bumpers
void bumperCB(const robot_specific_msgs::Bumper::ConstPtr& __bumper)
{
    // check each bumper

    cout << "bumper " << (int)__bumper->bumper << endl;

    static bool left_bumper_flag = false;
    static bool right_bumper_flag = false;

    // check left bumper
    if (((int)__bumper->bumper == 1) && ((int)__bumper->state == 1))
    {
        left_bumper_flag = !left_bumper_flag;   // toggle flag

        // do something, e.g.:
        // set / reset stiffness
        if (left_bumper_flag)
        {
          // tutorial 2
          // setStiffness(0.005, "Head");
          // setStiffness(0.005, "LArm");
          // setStiffness(0.005, "RArm");

          // tutorial 3
          setStiffness(0.005, "Head");
          float stiff[] = {0.0, 0.0, 0.9, 0.9, 0.8, 0.8};
          setStiffness(stiff, "LArm");
        }
    }


    // check right bumper
    if (((int)__bumper->bumper == 0) && ((int)__bumper->state == 1))
    {
        right_bumper_flag = !right_bumper_flag;     // toggle flag

        setStiffness(0.005, "Head");
        setStiffness(0.005, "LArm");
        setStiffness(0.005, "RArm");
    }

}


// callback function for the head joints
void jointStateCB(const robot_specific_msgs::JointState::ConstPtr& joint_state)
{
    // buffer for incoming message
    std_msgs::Float32MultiArray buffer;

    // index
    int idx;


    // extract the proprioceptive state of the HEAD
    buffer.data.clear();
    for (int i=0; i<ROBOT_DOF; i++)
    {
        if (joint_state->name[i] == "HeadYaw")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << ": " << joint_state->position[i] << endl;
        }
        if (joint_state->name[i] == "HeadPitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << ": " << joint_state->position[i] << endl;
        }
    }

    // write data into array
    idx = 0;
    for(vector<float>::const_iterator iter = buffer.data.begin(); iter != buffer.data.end(); ++iter)
    {
        // store into temporary target motor state buffer
        motor_head_in[idx] = *iter;
        idx++;
    }

    // display data on terminal
    // cout << "Head joints:  ";
    // for (int i=0; i<HEAD_DOF; i++)
    //     cout << motor_head_in[i] << " ";
    // cout << endl;


    // extract the proprioceptive state of the LEFT ARM
    buffer.data.clear();
    for (int i=0; i<ROBOT_DOF; i++)
    {
        if (joint_state->name[i] == "LShoulderPitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LShoulderRoll")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LElbowYaw")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LElbowRoll")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LWristYaw")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LHand")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }

    }

    // write data into array
    idx = 0;
    for(vector<float>::const_iterator iter = buffer.data.begin(); iter != buffer.data.end(); ++iter)
    {
        // store into temporary target motor state buffer
        motor_l_arm_in[idx] = *iter;
        idx++;
    }

    // display data on terminal
    // cout << "Left arm joints:  ";
    // for (int i=0; i<L_ARM_DOF; i++)
    //     cout << motor_l_arm_in[i] << " ";
    // cout << endl;


    // extract the proprioceptive state of the RIGHT ARM
    buffer.data.clear();
    for (int i=0; i<ROBOT_DOF; i++)
    {
        if (joint_state->name[i] == "RShoulderPitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RShoulderRoll")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RElbowYaw")
        {
           buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RElbowRoll")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RWristYaw")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RHand")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }

    }

    // write data into array
    idx = 0;
    for(vector<float>::const_iterator iter = buffer.data.begin(); iter != buffer.data.end(); ++iter)
    {
        // store into temporary target motor state buffer
        motor_r_arm_in[idx] = *iter;
        idx++;
    }

    // display data on terminal
    // cout << "Right arm joints:  ";
    // for (int i=0; i<R_ARM_DOF; i++)
    //     cout << motor_r_arm_in[i] << " ";
    // cout << endl;

}


// send commanded joint positions of the HEAD
void sendTargetJointStateHead(/* maybe a result as function argument */)
{
    double dummy[HEAD_DOF];  // dummy representing the comanded joint state
    robot_specific_msgs::JointAnglesWithSpeed target_joint_state;

    // specify the limb
    target_joint_state.joint_names.clear();
    target_joint_state.joint_names.push_back("Head");

    // specifiy the angle
    target_joint_state.joint_angles.clear();
    for (int i=0; i<HEAD_DOF; i++)
        target_joint_state.joint_angles.push_back(dummy[i] /* array containing result */);

    // set speed
    target_joint_state.speed = 0.2;

    // set the mode of joint change
    target_joint_state.relative = 0;

    // send to robot
    target_joint_state_pub.publish(target_joint_state);

}


// send commanded joint positions of the LEFT ARM
void sendTargetJointStateLArm(double dummy[])
{
    //double dummy[L_ARM_DOF];  // dummy representing the comanded joint state
    robot_specific_msgs::JointAnglesWithSpeed target_joint_state;

    // specify the limb
    target_joint_state.joint_names.clear();
    target_joint_state.joint_names.push_back("LArm");

    // specifiy the angle
    target_joint_state.joint_angles.clear();
    for (int i=0; i<L_ARM_DOF; i++)
        target_joint_state.joint_angles.push_back(dummy[i] /* array containing result */);

    // set speed
    target_joint_state.speed = 0.2;

    // set the mode of joint change
    target_joint_state.relative = 0;

    // send to robot
    target_joint_state_pub.publish(target_joint_state);

}


// send commanded joint positions of the RIGHT ARM
void sendTargetJointStateRArm(double dummy[])
{
    ///double dummy[R_ARM_DOF];  // dummy representing the comanded joint state
    robot_specific_msgs::JointAnglesWithSpeed target_joint_state;

    // specify the limb
    target_joint_state.joint_names.clear();
    target_joint_state.joint_names.push_back("RArm");

    // specifiy the angle
    target_joint_state.joint_angles.clear();
    for (int i=0; i<R_ARM_DOF; i++)
        target_joint_state.joint_angles.push_back(dummy[i] /* array containing result */);

    // set speed
    target_joint_state.speed = 0.2;

    // set the mode of joint change
    target_joint_state.relative = 0;

    // send to robot
    target_joint_state_pub.publish(target_joint_state);

}


// callback function for key events
void keyCB(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("key pushed: %s", msg->data.c_str());

    // start the robot behaviour
    if (*(msg->data.c_str()) == '0')
    {
        cout << "keyCB()" << endl;


    }

}

void InitHomePositions()
{
  /*  Left Arm */
  L_ARM_HomePosition.joint_names.clear();
  L_ARM_HomePosition.joint_names.push_back("LArm");
  L_ARM_HomePosition.joint_angles.clear();
  L_ARM_HomePosition.joint_angles.push_back(+0.00);
  L_ARM_HomePosition.joint_angles.push_back(+0.55);
  L_ARM_HomePosition.joint_angles.push_back(-0.20);
  L_ARM_HomePosition.joint_angles.push_back(-1.30);
  L_ARM_HomePosition.joint_angles.push_back(0);
  L_ARM_HomePosition.joint_angles.push_back(0);
  L_ARM_HomePosition.speed = 0.2;
  L_ARM_HomePosition.relative = 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "central_node");
    ros::NodeHandle central_node_nh;
    cout << "helloworld" << endl;

    // messaging with the NAO nodes

    // advertise joint stiffnesses
    stiffness_pub = central_node_nh.advertise<robot_specific_msgs::JointState>("joint_stiffness", 1);

    // subscribe to the joint states
    // the topic is the same as the one of the wrapper node of the NAO robot
    ros::Subscriber joint_state_sub;
    joint_state_sub = central_node_nh.subscribe("joint_states", 1, &jointStateCB);

    // advertise the target joint states
    target_joint_state_pub = central_node_nh.advertise<robot_specific_msgs::JointAnglesWithSpeed>("joint_angles", 1);    // to NAO robot

    // using image_transport to publish and subscribe to images
    image_transport::ImageTransport image_tran(central_node_nh);

    // subscribe to the raw camera image
    image_transport::Subscriber image_sub;
    image_sub = image_tran.subscribe("nao_robot/camera/top/camera/image_raw", 1, &visionCB);

    // subscribe to tactile and touch sensors
    tactile_sub = central_node_nh.subscribe("tactile_touch", 1, tactileCB);
    bumper_sub = central_node_nh.subscribe("bumper", 1, bumperCB);

    // set up the subscriber for the keyboard
    ros::Subscriber key_sub;
    key_sub = central_node_nh.subscribe("key", 5, keyCB);

    // waving publisher
    waving_pub = central_node_nh.advertise<std_msgs::String>("wave", 1);

    // create a GUI window for the raw camera image
    namedWindow(cam_window, WINDOW_AUTOSIZE);

    // call init home function for both arms
    // InitHomePositions();

    ros::spin();

    return 0;
}
