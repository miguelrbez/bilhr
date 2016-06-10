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

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "reinforcementlearning_node");
    ros::NodeHandle rl_node_nh;
    cout << "starting reinforcementlearning_node" << endl;

    ros::spin();

    return 0;
}
