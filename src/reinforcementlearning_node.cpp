/*
  * Team: Teamname
  * - Emilia Skurzynska
  * - Jiabin Lastname
  * - Miguel Rodriguez
  * - Martin Patz
  * - Adam Zylka
  *
  * HOW TO START
roslaunch nao_bringup nao_full_py.launch nao_ip:=10.152.246.242 roscore_ip:=10.152.246.128
roslaunch nao_apps tacticle.launch nao_ip:=10.152.246.242 roscore_ip:=10.152.246.128
roslaunch bl_group_e start.launch
*/

// ros includes
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include "std_msgs/String.h"

// cpp standard includes
#include <iostream>
#include <string>

// robot config
#include "robot_config.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

/***************************
* LOCAL DEFINITIONS
***************************/
// subscriber to keyboard
ros::Subscriber key_sub;

// subscribers to tactile and touch sensors
ros::Subscriber tactile_sub;
ros::Subscriber bumper_sub;

// joint stiffnesses
ros::Publisher stiffness_pub;

// reward
int reward = 0;


/***************************
* LOCAL - FUNCTIONS
***************************/
// converts int to string
string IntToStr(int a)
{
  stringstream ss;
  ss << a;
  return ss.str();
}

/***************************
* CALLBACK - FUNCTIONS
***************************/
// callback function for key events
void keyCB(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("rl_node received revard: %s", msg->data.c_str());

    // goal
    if (msg->data.c_str() == IntToStr(20))
      reward = 20;
    // move leg
    else if (msg->data.c_str() == IntToStr(-1))
      reward = -1;
    // miss goal or miss ball
    else if (msg->data.c_str() == IntToStr(-5))
      reward = -5;
    // fall
    else if (msg->data.c_str() == IntToStr(-20))
      reward = -20;

    // cout << "reward: " << reward << endl;
}

// callback function for tactile buttons (TBs) on the head
void tactileCB(const robot_specific_msgs::TactileTouch::ConstPtr& __tactile_touch)
{
    // check TB 3 (rear)
    if (((int)__tactile_touch->button == 3) && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;
    }

    // check TB 2 (middle)
    if (((int)__tactile_touch->button == 2) && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;
    }

    // check TB 1 (front)
    if (((int)__tactile_touch->button == 1) && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;
    }

}


// callback function for bumpers
void bumperCB(const robot_specific_msgs::Bumper::ConstPtr& __bumper)
{
    static bool left_bumper_flag = false;
    static bool right_bumper_flag = false;

    // check left bumper
    if (((int)__bumper->bumper == 1) && ((int)__bumper->state == 1))
    {
        left_bumper_flag = !left_bumper_flag;   // toggle flag
        cout << "pressed left bumper\n";
    }

    // check right bumper
    if (((int)__bumper->bumper == 0) && ((int)__bumper->state == 1))
    {
        right_bumper_flag = !right_bumper_flag;     // toggle flag
        cout << "pressed right bumper\n";
    }

}


/***************************
* MAIN
***************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "reinforcementlearning_node");
    ros::NodeHandle rl_node_nh;

    cout << "starting reinforcementlearning_node" << endl;

    // subscribe to keyboard
    key_sub = rl_node_nh.subscribe("key", 100, keyCB);

    // advertise joint stiffnesses
    stiffness_pub = rl_node_nh.advertise<robot_specific_msgs::JointState>("joint_stiffness", 1);

    // subscribe to tactile and touch sensors
    tactile_sub = rl_node_nh.subscribe("tactile_touch", 1, tactileCB);
    bumper_sub = rl_node_nh.subscribe("bumper", 1, bumperCB);

    ros::spin();

    return 0;
}
