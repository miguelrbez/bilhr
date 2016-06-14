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

// subscriber to goalstate
ros::Subscriber goalstate_sub;

// subscriber to legstate
ros::Subscriber legstate_sub;

// publisher for setting leg position
ros::Publisher set_leg_pos_pub;

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
    else
      printf("Hast n scheiss eingegeben: %s\n", msg->data.c_str());

    // cout << "reward: " << reward << endl;
}

// callback function for goalkeeper position
void goalstateCB(const std_msgs::String::ConstPtr& msg)
{
  // ROS_INFO("rl_node received goalstate: %s", msg->data.c_str());
}

// callback function for leg_state
void legstateCB(const std_msgs::String::ConstPtr& msg)
{
  // ROS_INFO("rl_node received legstate: %s", msg->data.c_str());

  // trigger leg position
  std_msgs::String msgSet;
  msgSet.data = "1";
  set_leg_pos_pub.publish(msgSet);
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

    // subscribe to goalstate
    goalstate_sub = rl_node_nh.subscribe("goalkeeper", 100, goalstateCB);

    // subscribe to legstate
    legstate_sub = rl_node_nh.subscribe("leg_state", 100, legstateCB);

    // advertise leg position
    set_leg_pos_pub = rl_node_nh.advertise<std_msgs::String>("set_leg_pos", 10);

    ros::spin();

    return 0;
}
