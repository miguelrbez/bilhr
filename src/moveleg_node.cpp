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
// subscribers to tactile and touch sensors
ros::Subscriber tactile_sub;
ros::Subscriber bumper_sub;

// subscribe to joint states
ros::Subscriber joint_state_sub;

// publisher to robot joints for sending the target joint angles
ros::Publisher target_joint_state_pub;

// publish joint stiffnesses
ros::Publisher stiffness_pub;

// publish leg state
ros::Publisher leg_state_pub;

// received motor state of the HEAD
double motor_head_in[HEAD_DOF];

// received motor state of the LEFT ARM
double motor_l_arm_in[L_ARM_DOF];

// received motor state of the RIGHT ARM
double motor_r_arm_in[R_ARM_DOF];

// received motor state of the LEFT LEG
double motor_l_leg_in[L_LEG_DOF];

// received motor state of the LEFT LEG
double motor_r_leg_in[R_LEG_DOF];


/***************************
* LOCAL FUNCTIONS
***************************/
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
  else if (name == "LLeg")
    dof = L_LEG_DOF;
  else if (name == "RLeg")
    dof = R_LEG_DOF;

  for (int i = 0; i < dof; i++)
    target_joint_stiffness.effort.push_back(value);

  stiffness_pub.publish(target_joint_stiffness);
}


// function for discretizing leg state and publishing it
void publish_legState_to_rl()
{
  std_msgs::String msg;

  double leg_state = motor_r_leg_in[R_HIP_ROLL];
  cout << "legstate: " << leg_state << endl;
  // discretize leg state
  // range of leg from -0.75 to 0.35
  // steps between: 0.11
  
  if (leg_state < -0.75)
    msg.data = "-1";
  else if (leg_state <= -0.64)
    msg.data = "1";
  else if (leg_state <= -0.53)
    msg.data = "2";
  else if(leg_state <= -0.42)
    msg.data = "3";
  else if(leg_state <= -0.31)
    msg.data = "4";
  else if(leg_state <= -0.2)
    msg.data = "5";
  else if(leg_state <= -0.9)
    msg.data = "6";
  else if(leg_state <= 0.02)
    msg.data = "7";
  else if(leg_state <= 0.13)
    msg.data = "8";
  else if(leg_state <= 0.24)
    msg.data = "9";
  else if(leg_state <= 0.35)
    msg.data = "10";

  leg_state_pub.publish(msg);
}


/***************************
* CALLBACK - FUNCTIONS
***************************/
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

        // set stiffness for head

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
    // for (int i=0; i<R_ARM_DOF; i++)// received motor state of the HEAD
    // double motor_head_in[HEAD_DOF];
    //     cout << motor_r_arm_in[i] << " ";
    // cout << endl;


    // extract the proprioceptive state of the LEFT LEG
    buffer.data.clear();
    for (int i=0; i<ROBOT_DOF; i++)
    {
        if (joint_state->name[i] == "LHipYawPitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LHipRoll")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LHipPitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LKneePitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LAnklePitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "LAnkleRoll")
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
        motor_l_leg_in[idx] = *iter;
        idx++;
    }


    // extract the proprioceptive state of the RIGHT LEG
    buffer.data.clear();
    for (int i=0; i<ROBOT_DOF; i++)
    {
        if (joint_state->name[i] == "RHipYawPitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RHipRoll")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RHipPitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RKneePitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RAnklePitch")
        {
            buffer.data.push_back(joint_state->position[i]);
            // cout << joint_state->name[i] << endl;
        }
        if (joint_state->name[i] == "RAnkleRoll")
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
        motor_r_leg_in[idx] = *iter;
        idx++;
    }

    // send leg state to rl_node
    publish_legState_to_rl();
}


/***************************
* MAIN
***************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveleg_node");
    ros::NodeHandle ml_node_nh;

    // subscribe to joint states
    joint_state_sub = ml_node_nh.subscribe("joint_states", 1, &jointStateCB);

    // advertise the target joint states
    target_joint_state_pub = ml_node_nh.advertise<robot_specific_msgs::JointAnglesWithSpeed>("joint_angles", 1);

    // advertise joint stiffnesses
    stiffness_pub = ml_node_nh.advertise<robot_specific_msgs::JointState>("joint_stiffness", 1);

    // advertise leg state
    leg_state_pub = ml_node_nh.advertise<std_msgs::String>("leg_state", 10);

    // subscribe to tactile and touch sensors
    tactile_sub = ml_node_nh.subscribe("tactile_touch", 1, tactileCB);
    bumper_sub = ml_node_nh.subscribe("bumper", 1, bumperCB);

    ros::spin();

    return 0;
}
