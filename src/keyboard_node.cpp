                            /*
    Author:         Erhard Wieser
    Description:    This node serves as an interface node, it publishes input from the keyboard.
		Modified by Adam Zylka
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <stdio.h>
#include <sstream>


using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard_node");

	ros::NodeHandle n;
	ros::Publisher pub;

	pub = n.advertise<std_msgs::Int32>("key", 10);

	ros::Rate loop_rate(10);     // publish with 10 Hz

	int count = 0;

	while (ros::ok())
	{
		// check keyboard
		std_msgs::Int32 ch;
		//ch.data = getchar();
		cin >> ch.data;

		pub.publish(ch); 	  // publish the keyboard input

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
