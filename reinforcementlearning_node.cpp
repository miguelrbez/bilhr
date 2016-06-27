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

string N_name ="N.txt";
string Q_name ="Q.txt";
string reward_name ="reward.txt";
string transition_name = "transitions.txt";

// Functions declaration
int read_writeRewards(string , int);
int read_writeTransition(string , int);
int read_writeNQ(string, int, int);

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
  bool operator ==(State a, State b) {
    return (a.leg_ang == b.leg_ang) && (a.keeper_dist == b.keeper_dist);
  }
  State(int a, int b){
    leg_ang = a;
    keeper_dist = b;
  }
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
 * Vector containing all allowed actions.
 */
vector<int> actions = {ACTION_MOVE_LEG_IN, ACTION_MOVE_LEG_OUT, ACTION_KICK};
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
const int nr_leg_bins = 10;
/**
 * Number of Goal Keeper (gk) bins.
 */
const int nr_gk_bins = 5;
/**
 * Number of available actions.
 */
const int nr_actions = 3;
/**
 * Number of neighbourhood states (left, same, right).
 */
const int nr_neighbour = 3;

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
 * 5D Matrix storing the transitions
 * 1. goal keeper state
 * 2. leg position state
 * 3. action
 * 4. number of neighbourhood states for 1.
 * 5. number of neighbouthood states for 2.
 */
int transition[nr_gk_bins][nr_leg_bins][nr_actions][nr_neighbour][nr_neighbour];
/**
 * Transitions of all the states.
 * 1. goal keeper state
 * 2. leg position state
 * 3. action
 */
int rewards[nr_gk_bins][nr_leg_bins][nr_actions];
/**
 * 3D matrix containing N - numbers of visits in each state-action pair.
 * 1. goal keeper state
 * 2. leg position state
 * 3. action
 */
int N[nr_gk_bins][nr_leg_bins][nr_actions];
/**
 * 3D matrix containing the Q function values. Vector levels:
 * 1. goal keeper state
 * 2. leg position state
 * 3. action
 */
int Q[nr_gk_bins][nr_leg_bins][nr_actions];

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
  int reward = stoi(msg->data.c_str());
  bool valid = find(begin(valid_rewards), end(valid_rewards), reward) != end(valid_rewards);
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
  current_state.keeper_dist = stoi(msg->data.c_str());
  ROS_INFO("Received  callback with value: %d", current_state.keeper_dist);
}

void lpCB(const std_msgs::String::ConstPtr& msg) {
  current_state.leg_ang = stoi(msg->data.c_str());
  ROS_INFO("Received  callback with value: %d", current_state.leg_ang);
}

/**
 * Returns the reward function for the 
 * @param   state   state from which the action was taken
 * @param   action  taken action
 */
double rewardFunction(State s, int action) {
  return rewards[s.keeper_dist][s.leg_ang][action];
}

/*
 * Beginning of functions defined by ES
 */

/**
 * Initializes the transition function in a manner specified by an user
 * @param type  for 0, the function initializes with a perfect scenario, 
 *        for 1, the function reads the values from file, by default with zeros
 */
void initTransitions(int type) {
  if (type == 0){
    cout<< "Default transition" <<endl;

    // For each state of the goal keeper
    for (int j = 0; j < nr_gk_bins; j++){
      // For each state of the leg
      for (int i = 0; i < nr_leg_bins; i++){
        // For each action
        for (int k = 0; k < nr_actions; k++){
          // For each possible neighbour
          for (int p = 0; p < nr_neighbour; p++){
            if (p == 1){
              if (k == ACTION_MOVE_LEG_OUT){
                transition[j][i][k][0][p] = 0;
                transition[j][i][k][1][p] = 0;
                transition[j][i][k][2][p] = 1;
              }
              if (k == ACTION_MOVE_LEG_IN){
                transition[j][i][k][0][p] = 1;
                transition[j][i][k][1][p] = 0;
                transition[j][i][k][2][p] = 0;
              }
              if (k == ACTION_KICK){
                transition[j][i][k][0][p] = 0;
                transition[j][i][k][1][p] = 1;
                transition[j][i][k][2][p] = 0;
              }
            }
            // Goal keeper doesn't change the position
            else{
              transition[j][i][k][0][p] = 0;
              transition[j][i][k][1][p] = 0;
              transition[j][i][k][2][p] = 0;
            }
          }
        }
      }
    }
  }
  else if (type == 1){
    read_writeTransition(transition_name, 0);
  }
  else{
    // For each state of the goal keeper
    for (int j = 0; j < nr_gk_bins; j++){
      // For each state of the leg
      for (int i = 0; i < nr_leg_bins; i++){
        // For each action
        for (int k = 0; k < nr_actions; k++){
          // For each outcome
          for (int p = 0; p < nr_neighbour; p++){
            transition[j][i][k][0][p] = 0;
            transition[j][i][k][1][p] = 0;
            transition[j][i][k][2][p] = 0;
          }
        }
      }
    }
  }
}

/**
 * Updates the transition model
 * @param   state   state from which the action was taken
 * @param   action  taken action
 * @param result  resulting state
 */
int updateTransitions(State state, int action, State result) {
  // Goal keeper moved left
  if (result.keeper_dist - state.keeper_dist == -1){
    if (result.leg_ang - state.leg_ang == -1)
      transition[state.keeper_dist][state.leg_ang][action][0][0] += 1;
    else if (result.leg_ang - state.leg_ang == 1)
      transition[state.keeper_dist][state.leg_ang][action][2][0] += 1;
    else if (result.leg_ang - state.leg_ang == 0)
      transition[state.keeper_dist][state.leg_ang][action][1][0] += 1;
  }
  // Goal keeper stayed on the same spot
  else if (result.keeper_dist - state.keeper_dist == 0){
    if (result.leg_ang - state.leg_ang == -1)
      transition[state.keeper_dist][state.leg_ang][action][0][1] += 1;
    else if (result.leg_ang - state.leg_ang == 1)
      transition[state.keeper_dist][state.leg_ang][action][2][1] += 1;
    else if (result.leg_ang - state.leg_ang == 0)
      transition[state.keeper_dist][state.leg_ang][action][1][1] += 1;
  }
  // Goal keeper moved right
  else if (result.keeper_dist - state.keeper_dist == 1){
    if (result.leg_ang - state.leg_ang == -1)
      transition[state.keeper_dist][state.leg_ang][action][0][2] += 1;
    else if (result.leg_ang - state.leg_ang == 1)
      transition[state.keeper_dist][state.leg_ang][action][2][2] += 1;
    else if (result.leg_ang - state.leg_ang == 0)
      transition[state.keeper_dist][state.leg_ang][action][1][2] += 1;
  }
  return 1;
}

/**
 * Returns the transition function (number of times the transition ) for the 
 * @param   state   state from which the action was taken
 * @param   action  taken action
 * @param result  resulting state
 */
vector<vector<double> > transitionFunction(State state, int action) {
  vector<vector<double> > Pr;
  vector<double> p;
  int sum = 0;
  for (int i=0; i < nr_neighbour; i++){
    for (int j=0; j < nr_neighbour; j++){
      sum += transition[state.keeper_dist][state.leg_ang][action][i][j];
    }
  }
  if (sum == 0)
    sum = 1;
  for (int i = 0; i < nr_neighbour; i++){
    p.clear();
    for (int j = 0; j < nr_neighbour; j++){
      p.push_back((double)transition[state.keeper_dist][state.leg_ang][action][i][j]/sum);
    }
    Pr.push_back(p);
  }
  return Pr;
}

/**
 * Reads or writes transition function from the given file. 
 * @param   filename  name of the file
 * @param   type    type of the action, 0-read, 1-write
 */
int read_writeTransition(string filename, int type) {
  if (type == 0){
    ofstream file;
      file.open (filename);
    // For each state of the goal keeper
    for (int j = 0; j < nr_gk_bins; j++){
      // For each state of the leg
      for (int i = 0; i < nr_leg_bins; i++){
        // For each action
        for (int k = 0; k < nr_actions; k++){
          // For each outcome
          for (int p = 0; p < nr_neighbour; p++){
            file << transition[j][i][k][p][0] << "\t";
            file << transition[j][i][k][p][1] << "\t";
            file << transition[j][i][k][p][2] << "\t";
          }
          file << endl;
        }
        file << endl;
      }
      file << endl;
    }
    file.close();
  }
  else if (type == 1){
    ifstream file_out;
      file_out.open (filename);
    // For each state of the goal keeper
    for (int j = 0; j < nr_gk_bins; j++){
      // For each state of the leg
      for (int i = 0; i < nr_leg_bins; i++){
        // For each action
        for (int k = 0; k < nr_actions; k++){
          // For each outcome
          for (int p = 0; p < nr_neighbour; p++){
            file_out >> transition[j][i][k][p][0];
            file_out >> transition[j][i][k][p][1];
            file_out >> transition[j][i][k][p][2];
          }
        }
      }
    }
    file_out.close();
  }
}

/**
 * Initializes the transition function in a manner specified by an user
 * @param type  for 0, the function initializes with a perfect scenario, 
 *        for 1, the function reads the values from file, by default with zeros
 */
void initRewards(int type) {
  if (type == 0){
    cout << "Default rewards" <<endl;
    // For each state of the goal keeper
    for (int j = 0; j < nr_gk_bins; j++){
      // For each state of the leg
      for (int i = 0; i < nr_leg_bins; i++){
        // For each action
        for (int k = 0; k < nr_actions; k++){
          if (k == ACTION_MOVE_LEG_OUT){
            rewards[j][i][k] = -1;
          }
          else if (k == ACTION_MOVE_LEG_IN){
            rewards[j][i][k] = -1;
          }
          else if (k == ACTION_KICK){
            if (j + i/2 == 0)
              rewards[j][i][k] = 20;
            else
              rewards[j][i][k] = -5;
          }
        }
      }
    }
  }
  else if (type == 1){
    read_writeRewards(reward_name, 0);
  }
  else{
    // For each state of the goal keeper
    for (int j = 0; j < nr_gk_bins; j++){
      // For each state of the leg
      for (int i = 0; i < nr_leg_bins; i++){
        // For each action
        for (int k = 0; k < nr_actions; k++){
          rewards[j][i][k] = 0;
        }
      }
    }
  }
}

/**
 * Updates the reward functions
 * @param   state   state from which the action was taken
 * @param   action  taken action
 * @param reward  resulting reward
 */
void updateRewards(State state, int action){
  int reward;
  cout << "Give the reward"<<endl;
  cin >> reward;
    rewards[state.keeper_dist][state.leg_ang][action] = reward;
}

/**
 * Returns the reward function for the 
 * @param   state   state from which the action was taken
 * @param   action  taken action
 */
double rewardFunction(State s, int action) {
  return rewards[s.keeper_dist][s.leg_ang][action];
}

/**
 * Reads or writes reward function from the given file. 
 * @param   filename  name of the file
 * @param   type    type of the action, 0-read, 1-write
 */
int read_writeRewards(string filename, int type) {
  if (type == 0){
    ofstream file;
      file.open (filename);
    // For each state of the goal keeper
    for (int j = 0; j < nr_gk_bins; j++){
      // For each state of the leg
      for (int i = 0; i < nr_leg_bins; i++){
        // For each action
        for (int k = 0; k < nr_actions; k++){
          file << rewards[j][i][k] << "\t";
        }
        file << endl;
      }
      file << endl;
    }
    file.close();
  }
  else if (type == 1){
    ifstream file_out;
      file_out.open (filename);
    // For each state of the goal keeper
    for (int j = 0; j < nr_gk_bins; j++){
      // For each state of the leg
      for (int i = 0; i < nr_leg_bins; i++){
        // For each action
        for (int k = 0; k < nr_actions; k++){
          file_out >> rewards[j][i][k];
        }
      }
    }
    file_out.close();
  }
}

/**
 * Initializes N and Q matrices
 * @param type  integer value representing the type of the initialization
 */
void initNQ(int type) {
  if (type == 0){

    read_writeNQ(N_name,0,0);
    read_writeNQ(Q_name,1,0);
  }
  else{
    // For each state of the goal keeper
    for (int j = 0; j < nr_gk_bins; j++){
      // For each state of the leg
      for (int i = 0; i < nr_leg_bins; i++){
        // For each action
        for (int k = 0; k < nr_actions; k++){
          N[j][i][k] = 0;
          Q[j][i][k] = 0;
        }
      }
    }
  }
}

/**
 * Updates N matrix
 * @param current_state   current state that the robot is in
 * @param action      the action taken by the robot
 */
void updateN(State current_state, int action) {
  // Increase the number of visits of the state
  N[current_state.leg_ang][current_state.keeper_dist][action] += 1;
}
/**
 * Reads or writes transition function from the given file. 
 * @param   filename  name of the file
 * @param matrix    type of the matrxi, 0-N, 1-Q
 * @param   type    type of the action, 0-read, 1-write
 */
int read_writeNQ(string filename, int matrix, int type) {
  if (type == 0){
    if (matrix == 0){
      ofstream file;
        file.open (filename);
      // For each state of the goal keeper
      for (int j = 0; j < nr_gk_bins; j++){
        // For each state of the leg
        for (int i = 0; i < nr_leg_bins; i++){
          // For each action
          for (int k = 0; k < nr_actions; k++){
            file << N[j][i][k] << "\t";
          }
          file << endl;
        }
        file << endl;
      }
      file.close();
    }
    else if (matrix == 1){
      ofstream file;
        file.open (filename);
      // For each state of the goal keeper
      for (int j = 0; j < nr_gk_bins; j++){
        // For each state of the leg
        for (int i = 0; i < nr_leg_bins; i++){
          // For each action
          for (int k = 0; k < nr_actions; k++){
            file << Q[j][i][k] << "\t";
          }
          file << endl;
        }
        file << endl;
      }
      file.close();
    }
  }
  else if (type == 1){
    if (matrix == 0){
      ifstream file_out;
        file_out.open (filename);
      // For each state of the goal keeper
      for (int j = 0; j < nr_gk_bins; j++){
        // For each state of the leg
        for (int i = 0; i < nr_leg_bins; i++){
          // For each action
          for (int k = 0; k < nr_actions; k++){
            file_out >> N[j][i][k];
          }
        }
      }
        file_out.close();
    }
    else if (matrix == 1){
      ifstream file_out;
        file_out.open (filename);
      // For each state of the goal keeper
      for (int j = 0; j < nr_gk_bins; j++){
        // For each state of the leg
        for (int i = 0; i < nr_leg_bins; i++){
          // For each action
          for (int k = 0; k < nr_actions; k++){
            file_out >> Q[j][i][k];
          }
        }
      }
        file_out.close();
    }
  }
}

/**
 * Initializes all variables - Q, N, reward and transition function
 */
void initVariables(void) {
  // Initialize the Transition functions
  initTransitions(0); // Use default values, 0 - perfect, 1 from file, 2 - zeros

  // Initialize the rewards
  initRewards(0); // Use default values, 0 - perfect, 1 from file, 2 - zeros

  // Initialize N and Q matrices
  initNQ(2); // Use default values, 0 - from file, default - zeros
}

/**
 * Saves all variables into the files - Q, N, reward and transition function
 */
void saveVariables(void) {
  // Write transition functions
  read_writeTransition(transition_name,1);

  // Write rewards
  read_writeRewards(reward_name,1);

  // Write N and Q matrices
  read_writeNQ(N_name,0,1);
}

double QFunction(State s, int action) {
  cout<<"State checked" << s.leg_ang <<"\t" <<s.keeper_dist << endl;

  // If action taken is the kick, then the state is assumed to be the goal state
  if (action == ACTION_KICK){
    cout <<"Kick"<<endl;
    Q[s.leg_ang][s.keeper_dist][action] = rewardFunction(s, action);
    return rewardFunction(s, action);
  }
  else{
    // Initialize the sum with zero
    double sum = 0;
    vector<double> q_actions;
    // Get all of the probabilities of transition
    vector<vector<double> > Pr = transitionFunction(s,action);
    // For each possible next state s'
    for (int i = 0; i < Pr.size(); i++){
      for (int j = 0; j < Pr[i].size(); j++){
        // Check if transition is possible
        if (Pr[i][j] > 0 && (s.leg_ang + i - 1) > 0 && (s.leg_ang + i - 1) < nr_leg_bins 
          && (s.keeper_dist + j - 1) > 0 && (s.keeper_dist + j - 1) < nr_gk_bins){
            q_actions.clear();
            // For each possible action
            for (int k = 0; k < nr_actions; k++){
              // Indexes of i and j - 0 (for -1), 1 (for 0), 2 (for 1)
              State next_s(s.leg_ang + i - 1,s.keeper_dist + j - 1);
              q_actions.push_back(QFunction(next_s,k));
            }
            // Choose the maximum value
            double max_q = *max_element(q_actions.begin(), q_actions.end());
            sum += Pr[i][j] * max_q;
        }
        else
          cout <<"Not possible" <<endl;
      }
    }
    // Update the Q value
    Q[s.leg_ang][s.keeper_dist][action] = sum;
    return sum;
  }
}

/*
 * End of functions defined by ES
 */

vector<State> genFutureStates(State s) {
  // Assumption: in the state transition, each x_i can only take the values of
  // it's direct neighbors.
  vector<State> new_states;
  State new_state;
  vector<int> delta = {-1, 0, +1};
  for (int it_lp = 0; it_lp < delta.size(); ++it_lp) { // iterator for leg position (it_lp)
    if (delta[it_lp] + s.leg_ang < 0 || delta[it_lp] + s.leg_ang >= nr_leg_bins)
      continue;
    for (int it_gk = 0; it_gk < delta.size(); ++it_gk) { // iterator for goal keeper (it_gp)
      if (delta[it_gk] + s.keeper_dist < 0 || delta[it_gk] + s.keeper_dist >= nr_gk_bins)
        continue;
      new_state.leg_ang = delta[it_lp] + s.leg_ang;
      new_state.keeper_dist = delta[it_gk] + s.keeper_dist;
      new_states.push_back(new_state);
    }
  }
  return new_states;
}

double qFunction(State s, int action) {
  double sum = 0;
  vector<State> fss = genFutureStates(s); // future states
  for(vector<State>::iterator fs_it = fss.begin(); fs_it != fss.end(); ++fs_it) {
    State fs = *fs_it;
    vector<double>::iterator max_el;
    vector<double> q_actions = Q[fs.keeper_dist][fs.leg_ang];
    double max_q = *max_element(q_actions.begin(), q_actions.end());
    sum += transitionFunction(s, fs, action) * max_q;
  }
  sum *= discount_factor;
  sum += rewardFunction(s, action);
  return sum;
}

void updatePolicy() {
  double max_el = 0;
  for (int gc = 0; gc < nr_gk_bins; gc++)  // goal keeper (gc)
    for (int lp = 0; lp < nr_leg_bins; lp++) { // leg position (lp)
      vector<double>::iterator max_el;
      max_el = max_element(Q[gc][lp].begin(), Q[gc][lp].end());
      policy[gc][lp] = actions[distance(Q[gc][lp].begin(), max_el)];
    }
}

vector<int> genPossibleMoves(State fs) {
  vector<int> fm = {ACTION_KICK};  // future moves
  if (fs.leg_ang > 0)
    fm.push_back(ACTION_MOVE_LEG_OUT);
  if (fs.leg_ang < nr_leg_bins)
    fm.push_back(ACTION_MOVE_LEG_IN);
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
