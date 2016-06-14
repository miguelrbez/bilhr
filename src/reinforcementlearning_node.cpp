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
#include <vector>
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
ros::Subscriber gs_sub;

// reward
int reward = 0;

struct State
{
  int leg_ang;      // range [0, num_angle_bins]
  int keeper_dist;  // range [0, num_keeper_dist_bins]
};

#define ACTION_MOVE_LEG_IN  = 0;
#define ACTION_MOVE_LEG_OUT = 1;
#define ACTION_KICK         = 2;

vector< vector<double> > Q;
vector< vector<int> > policy;

int nr_leg_bins = 10;
int nr_gc_bins = 5;

/**
 * The discount factor gamma for the Q function.
 * Value range: [0, 1]
 */
double discount_factor = 0.99;

/**
 * Threshold level to trigger, when exceeded, the exploitation mode. Before the
 * exploration mode is used.
 * Value range: [0, 1]
 */
double threshold_exploitation = 0.4;



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
}

/**
 * @brief      Callback function for goal state update cycle.
 *
 * @param[in]  msg   The message
 */
void gsCB(const std_msgs::String::ConstPtr& msg)
{
    // ROS_INFO("rl_node received gs state: %s", msg->data.c_str());
}

double rewardFunction(State s, int action) {
  // TODO: implement rewardFunction
}

double transitionFunction(State state, State future_state, int action) {
  // TODO: implement transitionFunction
}

double qFunction(State s, int action) {
  // TODO: implement qFunction
}

void updatePolicy() {
  // TODO: implement updatePolicy
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
    gs_sub = rl_node_nh.subscribe("goalkeeper",100, gsCB);


    ros::spin();

    return 0;
}
