/* setting stiffness via terminal
rosservice call /body_stiffness/disable "{}"
rosservice call /body_stiffness/enable "{}"
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

// subscribe to leg position
ros::Subscriber leg_state_sub;

// POSITION
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

// received position for right leg
double r_leg_pos = 0;

// vector of legparts for setting stiffness
vector<string> head_limb_names;
vector<string> left_arm_limbs_names;
vector<string> right_arm_limbs_names;
vector<string> left_leg_limbs_names;
vector<string> right_leg_limbs_names;


/***************************
* LOCAL FUNCTIONS
***************************/
// send commanded joint positions of the LEGS
void sendTargetJointState(string name, double dummy[])
{
  int repeat = 300;

  robot_specific_msgs::JointAnglesWithSpeed target_joint_state;

  for(int t = 0; t < repeat; t++)
  {
    // specify the limb
    for(int i = 0; i < L_LEG_DOF; i++)
    {
      // clear limb
      target_joint_state.joint_names.clear();
      target_joint_state.joint_angles.clear();

      // decide which leg
      if(name == "LLeg")
        target_joint_state.joint_names.push_back(left_leg_limbs_names[i]);
      else if(name == "RLeg")
        target_joint_state.joint_names.push_back(right_leg_limbs_names[i]);

      // set angle
      target_joint_state.joint_angles.push_back(dummy[i]);

      // set speed
      target_joint_state.speed = 0.2;

      // set the mode of joint change
      target_joint_state.relative = 0;

      // send to robot
      target_joint_state_pub.publish(target_joint_state);
    }
  }
}


// setting stiffness for different body limbs
void setStiffness(float value, std::string name)
{
  int repeat = 5000;
  cout << "setting stiffnesses for " << name << " to " << value << endl;
  robot_specific_msgs::JointState target_joint_stiffness;

  // set head stiffness
  if (name == "Head")
  {
    for(int t = 0; t < repeat; t++)
    {
      for(int i = 0; i < HEAD_DOF; i++)
      {
        target_joint_stiffness.name.clear();
        target_joint_stiffness.effort.clear();

        target_joint_stiffness.name.push_back(head_limb_names[i]);
        target_joint_stiffness.effort.push_back(value);

        stiffness_pub.publish(target_joint_stiffness);
      }
    }
  }

  // set arm stiffness
  else if (name == "LArm" || name == "RArm")
  {
    for(int t = 0; t < repeat; t++)
    {
      for(int i = 0; i < L_ARM_DOF; i++)
      {
        target_joint_stiffness.name.clear();
        target_joint_stiffness.effort.clear();

        // choose arm
        if(name == "LArm")
          target_joint_stiffness.name.push_back(left_arm_limbs_names[i]);
        else
          target_joint_stiffness.name.push_back(right_arm_limbs_names[i]);

        // set stiffness value
        target_joint_stiffness.effort.push_back(value);

        stiffness_pub.publish(target_joint_stiffness);
      }
    }
  }

  // set leg stiffness
  else if (name == "LLeg" || name == "RLeg")
  {
    for(int t = 0; t < repeat; t++)
    {
      for(int i = 0; i < L_LEG_DOF; i++)
      {
        target_joint_stiffness.name.clear();
        target_joint_stiffness.effort.clear();

        // choose leg
        if(name == "LLeg")
          target_joint_stiffness.name.push_back(left_leg_limbs_names[i]);
        else
          target_joint_stiffness.name.push_back(right_leg_limbs_names[i]);

        // set stiffness value
        target_joint_stiffness.effort.push_back(value);

        stiffness_pub.publish(target_joint_stiffness);
      }
    }
  }
}


// function for discretizing leg state and publishing it
void publish_legState_to_rl()
{
  std_msgs::Int32 msg;

  double leg_state = motor_r_leg_in[R_HIP_ROLL];

  // discretize leg state
  // range of leg from -0.75 to 0.35
  // steps between: 0.11

  if (leg_state < -0.75)
    msg.data = -1;
  else if (leg_state <= -0.64)
    msg.data = 1;
  else if (leg_state <= -0.53)
    msg.data = 2;
  else if(leg_state <= -0.42)
    msg.data = 3;
  else if(leg_state <= -0.31)
    msg.data = 4;
  else if(leg_state <= -0.2)
    msg.data = 5;
  else if(leg_state <= -0.9)
    msg.data = 6;
  else if(leg_state <= 0.02)
    msg.data = 7;
  else if(leg_state <= 0.13)
    msg.data = 8;
  else if(leg_state <= 0.24)
    msg.data = 9;
  else if(leg_state <= 0.35)
    msg.data = 10;

  leg_state_pub.publish(msg);
}


void standingOnOneLeg()
{
  cout << "standing on one leg in kicking pos\n";

  //TODO pose of right and left arm as well as head still missing

  double left_pos[] = {-0.0429101, 0.526204, -0.0475121, -0.0337899, 0.07359, 0.066004};
  sendTargetJointState("LLeg", left_pos);

  //TODO spann anpassen
  double right_pos[] = {-0.147222, 0.351328, 0.228524, 0.549214, -0.1733, -0.116542};
  sendTargetJointState("RLeg", right_pos);
}

void kick()
{
  cout << "kick\n";

  //TODO spann anpassen
  double kick_pose[] = {-0.00455999, 0.351328, -0.48632, 0.549214, -0.1733, -0.116542};
  sendTargetJointState("RLeg", kick_pose);
}

void adjustLeg()
{
  //TODO implement
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

        // kick function
        kick();
    }

    // check TB 2 (middle)
    if (((int)__tactile_touch->button == 2) && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;

        // standing on onle leg
        standingOnOneLeg();
    }

    // check TB 1 (front)
    if (((int)__tactile_touch->button == 1) && ((int)__tactile_touch->state == 1))
    {
        cout << "TB " << (int)__tactile_touch->button << " touched" << endl;

        // set stiffness for head and legs
        setStiffness(0.5, "Head");
        setStiffness(0.9, "LArm");
        setStiffness(0.9, "RArm");
        setStiffness(0.9, "LLeg");
        setStiffness(0.9, "RLeg");

        cout << "setting stiffness done\n";

        cout << "left leg pos:\n";
        for(int i = 0; i < L_LEG_DOF; i++)
          cout << motor_l_leg_in[i] << endl;

        cout << endl;

        cout << "right leg pos:\n";
        for(int i = 0; i < R_LEG_DOF; i++)
          cout << motor_r_leg_in[i] << endl;

        cout << endl;
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

        setStiffness(0.005, "Head");
        setStiffness(0.005, "LArm");
        setStiffness(0.005, "RArm");
        setStiffness(0.005, "LLeg");
        setStiffness(0.005, "RLeg");

        cout << "setting stiffness done\n";
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


// callback function for setting the leg position
void legStateCB(const std_msgs::Int32::ConstPtr& msg)
{
  // ROS_INFO("ml_node received legstate: %i", msg->data);

  if(msg->data == 1)
    r_leg_pos = -0.695;
  else if(msg->data == 2)
    r_leg_pos = -0.585;
  else if(msg->data == 3)
    r_leg_pos = -0.475;
  else if(msg->data == 4)
    r_leg_pos = -0.365;
  else if(msg->data == 5)
    r_leg_pos = -0.255;
  else if(msg->data == 6)
    r_leg_pos = -0.145;
  else if(msg->data == 7)
    r_leg_pos = -0.035;
  else if(msg->data == 8)
    r_leg_pos = 0.075;
  else if(msg->data == 9)
    r_leg_pos = 0.185;
  else if(msg->data == 10)
    r_leg_pos = 0.295;

  // cout << "r_leg_pos " << r_leg_pos << endl;
  // publish the leg position to the robot
  adjustLeg();
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
  leg_state_pub = ml_node_nh.advertise<std_msgs::Int32>("leg_state", 10);

  // subscribe to leg position
  leg_state_sub = ml_node_nh.subscribe("set_leg_pos", 100, legStateCB);

  // subscribe to tactile and touch sensors
  tactile_sub = ml_node_nh.subscribe("tactile_touch", 1, tactileCB);
  bumper_sub = ml_node_nh.subscribe("bumper", 1, bumperCB);

  // initialize head_limb names
  head_limb_names.push_back("HeadYaw");
  head_limb_names.push_back("HeadPitch");

  // initialize left_arm_limbs_names names
  left_arm_limbs_names.push_back("LShoulderPitch");
  left_arm_limbs_names.push_back("LShoulderRoll");
  left_arm_limbs_names.push_back("LElbowYaw");
  left_arm_limbs_names.push_back("LElbowRoll");
  left_arm_limbs_names.push_back("LWristYaw");
  left_arm_limbs_names.push_back("LHand");

  // initialize right_arm_limbs_names names
  right_arm_limbs_names.push_back("RShoulderPitch");
  right_arm_limbs_names.push_back("RShoulderRoll");
  right_arm_limbs_names.push_back("RElbowYaw");
  right_arm_limbs_names.push_back("RElbowRoll");
  right_arm_limbs_names.push_back("RWristYaw");
  right_arm_limbs_names.push_back("RHand");

  // initialize left_leg_limbs_names names
  left_leg_limbs_names.push_back("LHipYawPitch");
  left_leg_limbs_names.push_back("LHipRoll");
  left_leg_limbs_names.push_back("LHipPitch");
  left_leg_limbs_names.push_back("LKneePitch");
  left_leg_limbs_names.push_back("LAnklePitch");
  left_leg_limbs_names.push_back("LAnkleRoll");

  // initializr right_leg_limns names
  right_leg_limbs_names.push_back("RHipYawPitch");
  right_leg_limbs_names.push_back("RHipRoll");
  right_leg_limbs_names.push_back("RHipPitch");
  right_leg_limbs_names.push_back("RKneePitch");
  right_leg_limbs_names.push_back("RAnklePitch");
  right_leg_limbs_names.push_back("RAnkleRoll");

  ros::spin();

  return 0;
}
