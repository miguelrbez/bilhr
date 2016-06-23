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
 * - Adam
 */

// ros includes
// #include <ros/ros.h>
// #include <sensor_msgs/image_encodings.h>
// #include <sensor_msgs/Image.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <std_msgs/Int32.h>
// #include <vector>
// #include "std_msgs/String.h"

// cpp standard includes
#include <iostream>
#include <string>

// robot config
// #include "robot_config.h"

// test includes
#include <vector>
#include <algorithm>

using namespace std;
// using namespace sensor_msgs;
// using namespace message_filters;

/***************************
* LOCAL DEFINITIONS
***************************/
// // subscriber to keyboard
// ros::Subscriber key_sub;

// // subscriber to goalstate
// ros::Subscriber gs_sub;

struct State
{
  int leg_ang;      // range [0, num_angle_bins]
  int keeper_dist;  // range [0, num_keeper_dist_bins]
  // bool operator ==(State a, State b) {
  // 	return (a.leg_ang == b.leg_ang) && (a.keeper_dist == b.keeper_dist);
  // }
};

/**
 * Triggering this action will increment the angle by -1.
 */
int ACTION_MOVE_LEG_IN = 0;
/**
 * Triggering this action will increment the angle by +1.
 */
int ACTION_MOVE_LEG_OUT = 1;
/**
 * Triggering this action will make the robot kick and remain in the same
 * position.
 */
int ACTION_KICK = 2;

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
  * 3D matrix containing visited states
  * Vector levels:
  * 1. goal keeper state
  * 2. leg position state
  * 3. action
  */
vector<vector<vector<int> > > visitsMatrix;
/**
 * Vector containing all allowed actions.
 */
vector<int> actions;
/**
 * 3D Matrix storing the rewards for the state-action pairs
 */
vector< vector< vector<int> > > rewardsMatrix;

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

/**
 * Matrix which contains mean reward
 * Dimensions:
 * goalkeeper state
 * leg state
 * action
 */

vector<vector<vector<double> > > meanRewardMatrix;

/***************************
* LOCAL - FUNCTIONS
***************************/
// converts int to string
// string IntToStr(int a) {
//   stringstream ss;
//   ss << a;
//   return ss.str();
// }

/***************************
* CALLBACK - FUNCTIONS
***************************/
// callback function for key events
// void keyCB(const std_msgs::String::ConstPtr& msg) {
//   int reward = stoi(msg->data.c_str());
//   bool valid = find(begin(valid_rewards), end(valid_rewards), reward) != end(valid_rewards);
//   ROS_INFO("rl_node received revard: %d", reward);
//   if (valid)
//     ROS_INFO("Reward not valid!");
//   rewards[current_state.keeper_dist][current_state.leg_ang][current_action] = reward;
// }

// /**
//  * @brief      Callback function for goal state update cycle.
//  *
//  * @param[in]  msg   The message
//  */
// void gsCB(const std_msgs::String::ConstPtr& msg) {
//   current_state.keeper_dist = stoi(msg->data.c_str());
//   ROS_INFO("Received  callback with value: %d", current_state.keeper_dist);
// }

// void lpCB(const std_msgs::String::ConstPtr& msg) {
//   current_state.leg_ang = stoi(msg->data.c_str());
//   ROS_INFO("Received  callback with value: %d", current_state.leg_ang);
// }

// double rewardFunction(State s, int action) {
//   return rewards[s.keeper_dist][s.leg_ang][action];
// }

// double transitionFunction(State state, State future_state, int action) {
//   return 0.0;
// }

// vector<State> genFutureStates(State s) {
//   // Assumption: in the state transition, each x_i can only take the values of
//   // it's direct neighbors.
//   vector<State> new_states;
//   State new_state;
//   vector<int> delta = {-1, 0, +1};
//   for (int it_lp = 0; it_lp < delta.size(); ++it_lp) { // iterator for leg position (it_lp)
//     if (delta[it_lp] + s.leg_ang < 0 || delta[it_lp] + s.leg_ang >= nr_leg_bins)
//       continue;
//     for (int it_gk = 0; it_gk < delta.size(); ++it_gk) { // iterator for goal keeper (it_gp)
//       if (delta[it_gk] + s.keeper_dist < 0 || delta[it_gk] + s.keeper_dist >= nr_gk_bins)
//         continue;
//       new_state.leg_ang = delta[it_lp] + s.leg_ang;
//       new_state.keeper_dist = delta[it_gk] + s.keeper_dist;
//       new_states.push_back(new_state);
//     }
//   }
//   return new_states;
// }

// double qFunction(State s, int action) {
//   double sum = 0;
//   vector<State> fss = genFutureStates(s); // future states
//   for(vector<State>::iterator fs_it = fss.begin(); fs_it != fss.end(); ++fs_it) {
//     State fs = *fs_it;
//     vector<double>::iterator max_el;
//     vector<double> q_actions = Q[fs.keeper_dist][fs.leg_ang];
//     double max_q = *max_element(q_actions.begin(), q_actions.end());
//     sum += transitionFunction(s, fs, action) * max_q;
//   }
//   sum *= discount_factor;
//   sum += rewardFunction(s, action);
//   return sum;
// }

// void updatePolicy() {
//   double max_el = 0;
//   for (int gc = 0; gc < nr_gk_bins; gc++)  // goal keeper (gc)
//     for (int lp = 0; lp < nr_leg_bins; lp++) { // leg position (lp)
//       vector<double>::iterator max_el;
//       max_el = max_element(Q[gc][lp].begin(), Q[gc][lp].end());
//       policy[gc][lp] = actions[distance(Q[gc][lp].begin(), max_el)];
//     }
// }

vector<int> genPossibleMoves(State fs) {
  vector<int> fm = {ACTION_KICK};  // future moves
  if (fs.leg_ang > 0)
    fm.push_back(ACTION_MOVE_LEG_OUT);
  if (fs.leg_ang < nr_leg_bins)
    fm.push_back(ACTION_MOVE_LEG_IN);

  return fm;
}


// function for rewarding the robot
void rewardForRobot(State s, int a)
{
  double tmp;
  bool correct = false;

  // repeat as long as the user does not enter the valid reward
  while(!correct)
  {
    cout << "possible valid_rewards: '-1', '-5', '+20'" << endl;
    cout << "reward: ";
    
    cin >> tmp;


    for(int i = 0; i < valid_rewards.size(); i++)
      if(tmp == valid_rewards[i])
        correct = true;

    // reward only if correct
    if(correct)
      rewardsMatrix[s.keeper_dist-1][s.leg_ang-1][a] = tmp;
    else
      cout << "wrong reward - try again!!!\n";
  }
}


// update visits
void updateVisits(State s, int a)
{
  visitsMatrix[s.keeper_dist-1][s.leg_ang-1][a]++;
}


// get nummber of visits
int getVisits(State s, int a)
{
  return visitsMatrix[s.keeper_dist-1][s.leg_ang-1][a];
}


// init all needed matrices and variables
void initVariables() {
  // actions
  actions = {ACTION_MOVE_LEG_IN, ACTION_MOVE_LEG_OUT, ACTION_KICK};

  // nr of actions
  nr_actions = actions.size();

  // matrices have to be created in an reverse order

  // mean reward matrix
  // 1. dimension
  vector<double> vd1(nr_actions);
  // 2. dimension
  vector<vector<double> > vd2;
  for(int i = 0; i < nr_leg_bins; i++)
    vd2.push_back(vd1);
  // 3. dimension
  for(int i = 0; i < nr_gk_bins; i++)
    meanRewardMatrix.push_back(vd2);

  // check
  cout << "mean reward matrix size: " << meanRewardMatrix.size() << "x" << meanRewardMatrix[0].size() << "x" << 
    meanRewardMatrix[0][0].size() << endl;


  // rewardsMatrix
  // 1. dimension
  vector<int> vi1;
  // 2. dimension
  vector<vector<int> > vi2;
  for(int i = 0; i < nr_leg_bins; i++)
    vi2.push_back(vi1);
  // 3. dimension
  for(int i = 0; i < nr_gk_bins; i++)
    rewardsMatrix.push_back(vi2);

  // check
  cout << "reward matrix size: " << rewardsMatrix.size() << "x" << rewardsMatrix[0].size() << "x" << 
    rewardsMatrix[0][0].size() << endl;

  // visitsMatrix
  // clear vectors
  vi1.clear();
  vi2.clear();
  // 1. dimension
  vi1.resize(nr_actions);
  // 2. dimension
  for(int i = 0; i < nr_leg_bins; i++)
    vi2.push_back(vi1);
  // 3. dimension
  for(int i = 0; i < nr_gk_bins; i++)
    visitsMatrix.push_back(vi2);

  // check
  cout << "visits matrix size: " << visitsMatrix.size() << "x" << visitsMatrix[0].size() << "x" << 
    visitsMatrix[0][0].size() << endl;
}

/***************************
* MAIN
***************************/
int main(int argc, char** argv) 
{
  // variables
  bool loop = true;

  int a;
  State s;
  // init state
  s.keeper_dist = 1;
  s.leg_ang = 1;

  initVariables();

  while(loop)
  {
    // only testing
    for(int i = 0; i < 5; i++)
    {
      cout << "loop " << i << endl;

      // check visits
      updateVisits(s, a);
      cout << "number of visits: " << getVisits(s, a) << endl;

      // check reward

    }


    loop = false;
  }

  cout << endl;
  cout << "finished\n";

  // ros::init(argc, argv, "reinforcementlearning_node");
  // ros::NodeHandle rl_node_nh;

  // cout << "starting reinforcementlearning_node" << endl;

  // // subscribe to keyboard
  // key_sub = rl_node_nh.subscribe("key", 100, keyCB);

  // // subscribe to goalstate
  // gs_sub = rl_node_nh.subscribe("goalkeeper",100, gsCB);

  // initVariables();

  // ros::spin();

  return 0;
}
