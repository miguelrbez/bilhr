// /*
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
#include <fstream>
#include <algorithm>

// robot config
#include "robot_config.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

string N_name ="/home/bilhrgroup_e/Documents/N.txt";
string Q_name ="/home/bilhrgroup_e/Documents/Q.txt";
string reward_name ="/home/bilhrgroup_e/Documents/reward.txt";
string transition_name = "/home/bilhrgroup_e/Documents/transitions.txt";

/***************************
* LOCAL DEFINITIONS
***************************/
// subscriber to keyboard
ros::Subscriber key_sub;

// subscriber to goalstate
ros::Subscriber goalstate_sub;

// subscriber to legstate
ros::Subscriber legstate_sub;

// publisher of leg state
ros::Publisher set_leg_pos_pub;

struct State
{
  int leg_ang;      // range [0, num_angle_bins]
  int keeper_dist;  // range [0, num_keeper_dist_bins]
  bool operator ==(State b) {
    return (this->leg_ang == b.leg_ang) && (this->keeper_dist == b.keeper_dist);
  }
  State(int a, int b){
    leg_ang = a;
    keeper_dist = b;
  }
  State()
  {
    leg_ang = 0;
    keeper_dist = 0;
  }
};

// current state
State current_state;

// current action
int current_action = 1;


/**
 * 2D matrix containing the recommended actions for all state combinations.
 * Vector levels:
 * 1. goal keeper state
 * 2. leg position
 */
vector< vector<int> > policy;

int ACTION_MOVE_LEG_IN = 0;
int ACTION_MOVE_LEG_OUT = 1;
int ACTION_KICK = 2;
/**
 * Vector containing all allowed actions.
 */
vector<int> actions = {ACTION_MOVE_LEG_IN, ACTION_MOVE_LEG_OUT, ACTION_KICK};

/**
 * Array containing all valid rewards.
 */
int valid_rewards[4] = {-1, -5, -20, 20};
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

// MAX and MIN leg state
#define MAX_LEG_STATE 10
#define MIN_LEG_STATE 1

// leg state
int leg_state = 1;
/*
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

// Functions declaration
int read_writeRewards(string , int);
int read_writeTransition(string , int);
int read_writeNQ(string, int, int);
void updateN(State current_state, int action);
void updateRewards(State state, int action);
double QFunction(State s, int action);
void saveVariables(void);
int updateTransitions(State state, int action, State result);


/***************************
* CALLBACK - FUNCTIONS
***************************/
// callback function for key events
void keyCB(const std_msgs::Int32::ConstPtr& msg) {
  int reward = msg->data;
  State s = current_state;
  if(reward != 7 && reward != 8 && reward != 9)
  {
    bool valid = find(begin(valid_rewards), end(valid_rewards), reward) != end(valid_rewards);
    ROS_INFO("rl_node received revard: %d", reward);
    if (valid)
      ROS_INFO("Reward not valid!");
    rewards[current_state.keeper_dist][current_state.leg_ang][current_action] = reward;
    updateN(current_state, current_action);
    QFunction(current_state,current_action);
    saveVariables();
  }
  // for manual setting of the robot action
  else
  {
    if (msg->data == 9)
    {
      std_msgs::Int32 msgSet;
      msgSet.data = ACTION_MOVE_LEG_OUT;
      set_leg_pos_pub.publish(msgSet);
      ros::Duration(1).sleep();
      State s_prime = current_state;
      updateTransitions(s, msgSet.data, s_prime);
      saveVariables();
    }
    else if (msg->data == 8)
    {
      std_msgs::Int32 msgSet;
      msgSet.data = ACTION_KICK;
      set_leg_pos_pub.publish(msgSet);
      ros::Duration(1).sleep();
      State s_prime = current_state;
      updateTransitions(s, msgSet.data, s_prime);
      saveVariables();
    }
    else if (msg->data == 7)
    {
      std_msgs::Int32 msgSet;
      msgSet.data = ACTION_MOVE_LEG_IN;
      set_leg_pos_pub.publish(msgSet);
      ros::Duration(1).sleep();
      State s_prime = current_state;
      updateTransitions(s, msgSet.data, s_prime);
      saveVariables();
    }
  }

}

/**
 * @brief      Callback function for goal state update cycle.
 *
 * @param[in]  msg   The message
 */
void gsCB(const std_msgs::Int32::ConstPtr& msg) {
  current_state.keeper_dist = msg->data;
  ROS_INFO("Received  callback goal keeper with value: %d", current_state.keeper_dist);
}

void lpCB(const std_msgs::Int32::ConstPtr& msg) {
  current_state.leg_ang = msg->data;
  ROS_INFO("Received  callback leg position with value: %d", current_state.leg_ang);
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
    read_writeTransition(transition_name, 1);
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
    read_writeRewards(reward_name, 1);
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
  updateRewards(current_state, current_action);

void updateRewards(State state, int action){
  int reward;
  cout << "Give the reward"<<endl;
  cin >> reward;
    rewards[state.keeper_dist][state.leg_ang][action] = reward;
}
 */

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

    read_writeNQ(N_name,0,1);
    read_writeNQ(Q_name,1,1);
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
        file.open(filename);
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
      cout<< "Writing Q"<<endl;
      // For each state of the goal keeper
      for (int j = 0; j < nr_gk_bins; j++){
        // For each state of the leg
        for (int i = 0; i < nr_leg_bins; i++){
          // For each action
          for (int k = 0; k < nr_actions; k++){
            cout << Q[j][i][k] << "\t";
          }
          cout<<endl;
        }
        cout<<endl;
      }
      ofstream fileQ;
        fileQ.open(filename);
      // For each state of the goal keeper
      for (int j = 0; j < nr_gk_bins; j++){
        // For each state of the leg
        for (int i = 0; i < nr_leg_bins; i++){
          // For each action
          for (int k = 0; k < nr_actions; k++){
            fileQ << Q[j][i][k] << "\t";
          }
          fileQ << endl;
        }
        fileQ << endl;
      }
      fileQ.close();
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
  initNQ(0); // Use default values, 0 - from file, default - zeros
}

/**
 * Saves all variables into the files - Q, N, reward and transition function
 */
void saveVariables(void) {
  // Write transition functions
  read_writeTransition(transition_name,0);

  // Write rewards
  read_writeRewards(reward_name,0);

  // Write N and Q matrices
  read_writeNQ(N_name,0,0);
  read_writeNQ(Q_name,1,0);
}

double QFunction(State s, int action) {
  // If action taken is the kick, then the state is assumed to be the goal state
  if (action == ACTION_KICK){
    Q[s.keeper_dist][s.leg_ang][action] = rewardFunction(s, action);
    cout <<"Reward function" <<rewardFunction(s, action) <<endl;
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
        if (Pr[i][j] > 0 && (s.leg_ang + i - 1) >= 0 && (s.leg_ang + i - 1) < nr_leg_bins
          && (s.keeper_dist + j - 1) >= 0 && (s.keeper_dist + j - 1) < nr_gk_bins){
            q_actions.clear();
            // For each possible action
            for (int k = 0; k < nr_actions; k++){
              // Indexes of i and j - 0 (for -1), 1 (for 0), 2 (for 1)
              State next_s(s.leg_ang + i - 1,s.keeper_dist + j - 1);
              q_actions.push_back(Q[next_s.keeper_dist][next_s.leg_ang][k]);//QFunction(next_s,k));
            }
            // Choose the maximum value
            double max_q = *max_element(q_actions.begin(), q_actions.end());
            sum += Pr[i][j] * max_q;
        }
        else
          cout << "Not possible" <<endl;
      }
    }
    // Update the Q value
    Q[s.keeper_dist][s.leg_ang][action] = sum;
    cout << "Sum calculated" << sum <<endl;
    cout << "QSum calculated" << Q[s.keeper_dist][s.leg_ang][action] <<endl;
    return sum;
  }
}
/*
 * End of functions defined by ES
 */


double get_max(double array[])
// return the max value of an array
{
  double temp=-1000;
  for(int i=0;i<(sizeof(array)/sizeof(array[0]));i++)
  {
    if (array[i]>temp)
       temp=array[i];
  }
  return temp;
}

double get_min(double array[])
// return the min value of an array
{
  double temp=1000;
  for(int i=0;i<(sizeof(array)/sizeof(array[0]));i++)
  {
    if (array[i]<temp)
       temp=array[i];
  }
  return temp;
}


int argmax_a(State s)
{
  /*
  int ACTION_MOVE_LEG_IN = 0;
  int ACTION_MOVE_LEG_OUT = 1;
  int ACTION_KICK = 2;
  int N[nr_gk_bins][nr_leg_bins][nr_actions];
  */
  double Q_opt[3];
  bool explore=false;
  int action_opt=0;

  for (int i = 0; i < 3; i++)
  {
    Q_opt[i]=QFunction(s, i);
  }

   if(get_max(Q_opt)>0.4*get_max((double*)valid_rewards))
    explore=false;
   else
    explore=true;

   if (explore==true)
   {
     // get the lowest nr_actions
     double nr_min=get_min((double*)N[s.keeper_dist][s.leg_ang]);
     for (int i = 0; i < 3; i++)
      {
        if (nr_min==get_min((double*)N[s.keeper_dist][s.leg_ang]))
        action_opt=i;
        break;
      }

   }
   else // exploitation
   {
     for (int i = 0; i < 3; i++)
      {
        if (Q_opt[i]==get_max(Q_opt))
        action_opt=i;
        break;
      }
   }
   return action_opt;
}

int get_policy(State s)
//get optimal action of state s
{
  int action_opt=argmax_a(s);
  return action_opt;
}





/***************************
* MAIN
***************************/
int main(int argc, char** argv)
{
    initVariables();

    ros::init(argc, argv, "reinforcementlearning_node");
    ros::NodeHandle rl_node_nh;

    // subscribe to keyboard
    key_sub = rl_node_nh.subscribe("key", 100, keyCB);

    // subscribe to goalstate
    goalstate_sub = rl_node_nh.subscribe("goalkeeper", 100, gsCB);

    // subscribe to legstate
    legstate_sub = rl_node_nh.subscribe("leg_state", 100, lpCB);

    // advertise leg position
    set_leg_pos_pub = rl_node_nh.advertise<std_msgs::Int32>("set_leg_pos", 10);

    ros::spin();

    return 0;
}
