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

/*
 * Responsible for reinforement learning algorithm:
 * - Jiabin
 * - Martin
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

struct State
{
  int leg_ang;      // range [0, num_angle_bins]
  int keeper_dist;  // range [0, num_keeper_dist_bins]
};

#define ACTION_MOVE_LEG_IN  = 0;
#define ACTION_MOVE_LEG_OUT = 1;
#define ACTION_KICK         = 2;

/**
 * 3D matrix containing the Q function values. Vector levels:
 * 1. goal keeper state
 * 2. leg position state
 * 3. action
 */
vector< vector< vector <double> > > Q;
/**
 * 2D matrix containing the recommended actions for all state combinations.
 * Vector levels:
 * 1. goal keeper state
 * 2. leg position
 */
vector< vector<int> > policy;
/**
 * Vector containing all allowed actions.
 */
vector<int> actions;
/**
 * 3D Matrix storing the rewards for the state-action pairs
 */
vector< vector< vector<int> > > rewards;

/**
 * Array containing all valid rewards.
 */
vector<int> valid_rewards = {-1, -5, -20, +20};
/**
 * Number of bins for the robot's leg position.
 */
int nr_leg_bins = 10;
/**
 * Number of Goal Keeper (gk) bins.
 */
int nr_gk_bins = 5;
/**
 * Number of available actions.
 */
int nr_actions;

State current_state;
int current_action;

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
string IntToStr(int a) {
  stringstream ss;
  ss << a;
  return ss.str();
}

/***************************
* CALLBACK - FUNCTIONS
***************************/
// callback function for key events
void keyCB(const std_msgs::String::ConstPtr& msg) {
  int reward = std::stoi(msg->data.c_str());
  bool valid = std::find(std::begin(valid_rewards), std::end(valid_rewards), reward) != std::end(valid_rewards);
  ROS_INFO("rl_node received revard: %d", reward);
  if (valid)
    ROS_INFO("Reward not valid!");
  rewards[current_state.keeper_dist][current_state.leg_ang][current_action] = reward;
}

/**
 * @brief      Callback function for goal state update cycle.
 *
 * @param[in]  msg   The message
 */
void gsCB(const std_msgs::String::ConstPtr& msg) {
  current_state.keeper_dist = std::stoi(msg);
  ROS_INFO("Received  callback with value: %d", current_state.keeper_dist);
}

void lpCB(const std_msgs::String::ConstPtr& msg) {
  current_state.leg_ang = std::stoi(msg);
  ROS_INFO("Received  callback with value: %d", current_state.leg_ang);
}

double rewardFunction(State s, int action) {
  return rewards[s.keeper_dist][s.leg_ang][action];
}

double transitionFunction(State state, State future_state, int action) {
  // TODO: implement transitionFunction
}

double qFunction(State s, int action) {
  double sum = 0;
  vector<state> fss = genFutureStates(s); // future states 
  for(vector<state>::iterator fs_it = fss.begin(); fs_it != fss.end(); ++fs_it) {
    State fs = *fs_it;
    vector<double>::iterator max_el;
    max_q = std::max_element(Q[fs.keeper_dist][fs.leg_ang].begin(), Q[fs.keeper_dist][fs.leg_ang].end());
    sum += transitionFunction(s, fs, action) * max_q;
  }
  sum *= gamma;
  sum += rewardFunction(s, action);
  return sum;
}

void updatePolicy() {
  double max_el = 0;
  for (int gc = 0; gc < nr_gk_bins; gc++)  // goal keeper (gc)
    for (int lp = 0; lp < nr_leg_bins; lp++) { // leg position (lp)
      vector<double>::iterator max_el;
      max_el = std::max_element(Q[gc][lp].begin(), Q[gc][lp].end());
      policy[gc][lp] = actions[distance(v.begin(), max_el)];
    }
}

vector<State> genFutureStates(State s) {
  // Assumption: in the state transition, each x_i can only take the values of
  // it's direct neighbors.
  vector<State> new_states;
  State new_state;
  vector<int> delta = {-1, 0, +1};
  for (int it_lp = 0; it_lp < delta.size(); ++it_lp) { // iterator for leg position (it_lp)
    if (delta[it_lp] + s.leg_ang < 0 || delta[it_lp] + s.leg_ang >= nr_leg_bins)
      continue;
    for (int it_gk = 0; it_gk < delta.sit_gkze(); ++it_gk) { // iterator for goal keeper (it_gp)
      if (delta[it_gk] + s.keeper_dist < 0 || delta[it_gk] + s.keeper_dist >= nr_gk_bins)
        continue;
      new_state.leg_ang = delta[it_lp] + s.leg_ang;
      new_state.keeper_dist = delta[it_gk] + s.keeper_dist;
      new_states.push_back(new_state);
    }
  }
  return new_states;
}

void initVariables() {
  vector<int> actions = {ACTION_MOVE_LEG_IN, ACTION_MOVE_LEG_OUT, ACTION_KICK};
  nr_actions = actions.size();
  vector<vector<vector<double>>> Q (nr_gk_bins, vector<vector<int>>(nr_leg_bins, vector<int>(nr_actions)) );
  vector< vector<int> > policy (nr_gk_bins, vector<int> (nr_leg_bins) );
}

/***************************
* MAIN
***************************/
int main(int argc, char** argv) {
  ros::init(argc, argv, "reinforcementlearning_node");
  ros::NodeHandle rl_node_nh;

  cout << "starting reinforcementlearning_node" << endl;

  // subscribe to keyboard
  key_sub = rl_node_nh.subscribe("key", 100, keyCB);

  // subscribe to goalstate
  gs_sub = rl_node_nh.subscribe("goalkeeper",100, gsCB);

  initVariables();

  ros::spin();

  return 0;
}
