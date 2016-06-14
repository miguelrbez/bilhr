/*
 *  Author:         Erhard Wieser
 *
    Description:    Robot-specific parameters: NAO robot.
*/

// NAO robot-specific h-files for the joint msgs.
#include <robot_specific_msgs/JointState.h>
#include <robot_specific_msgs/JointAnglesWithSpeed.h>
#include <robot_specific_msgs/TactileTouch.h>
#include <robot_specific_msgs/Bumper.h>

#define ROBOT_DOF       26	// robot-specific; NAO robot has 26 DOF in total
#define HEAD_DOF        2
#define L_ARM_DOF 	6
#define L_LEG_DOF       6
#define R_LEG_DOF	6
#define R_ARM_DOF	6
#define L_LEG_DOF 6
#define R_LEG_DOF 6

/*  Head  */
#define HEAD_YAW              0
#define HEAD_PITCH            1
/*  Left Arm  */
#define L_ARM_SHOULDER_PITCH  2 // +0.00    -0.05
#define L_ARM_SHOULDER_ROLL   3 // +0.55    +0.8
#define L_ARM_ELBOW_YAW       4 // -0.20    -0.75
#define L_ARM_ELBOW_ROLL      5 // -1.30    -0.75
#define L_ARM_WRIST_YAW       6
#define L_ARM_HAND            7
/*  Right Arm   */
#define R_ARM_SHOULDER_PITCH  20
#define R_ARM_SHOULDER_ROLL   21
#define R_ARM_ELBOW_YAW       22
#define R_ARM_ELBOW_ROLL      23
#define R_ARM_WRIST_YAW       24
#define R_ARM_HAND            25

// left shoulder
#define L_SHOULDER_PITCH_NEG_VALUE  2.0857
#define L_SHOULDER_ROLL_NEG_VALUE   0.3142
#define L_SHOULDER_PITCH_RANGE      4.1714
#define L_SHOULDER_ROLL_RANGE       1.6407

#define R_HIP_ROLL  1
