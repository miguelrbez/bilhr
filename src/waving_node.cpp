/*
  nao waving with hands
*/

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <sstream>

#include "robot_config.h"

ros::Publisher target_joint_state_pub;

// flags for moving arms
bool waving_left = false;
bool waving_both = false;

void waveCB(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("waveCB");

	// middle pressed
	if (*(msg->data.c_str()) == 'm')
	{
		// invert the flag for waving with the left arm
		waving_left = !waving_left;
		waving_both = false;
	}
	// back pressed
	else if (*(msg->data.c_str()) == 'b')
	{
		// invert the flag for waving with both arms
		waving_both = !waving_both;
	}
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


int main(int argc, char **argv)
{
	double lHomePos[] = {0.0, 	0.55, 	-0.20, -1.30, 0.0, 0.0};
	double rHomePos[] = {0.0, 	-0.55, 	0.20,  1.30, 0.0, 0.0};
	double lWavePos[] = {-0.05, 0.8, 		-0.75, -0.75, 0.0, 0.0};
	double rWavePos[] = {-0.05, -0.8, 	0.75,  0.75, 0.0, 0.0};

	ros::init(argc, argv, "waving_node");

	ros::NodeHandle n;

	ros::Subscriber wave = n.subscribe("wave", 1, waveCB);

	// publish arm movement
	target_joint_state_pub = n.advertise<robot_specific_msgs::JointAnglesWithSpeed>("joint_angles", 1);

	ros::Rate loop_rate(1);     // every 1 sec

	int count = 0;

	while (ros::ok())
	{
		if(waving_left)
		{
			sendTargetJointStateLArm(lWavePos);
			if(waving_both)
				sendTargetJointStateRArm(rWavePos);

			ros::spinOnce();
			loop_rate.sleep();

			sendTargetJointStateLArm(lHomePos);
			if(waving_both)
				sendTargetJointStateRArm(rHomePos);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
