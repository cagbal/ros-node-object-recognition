
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>

ros::Publisher pub, pub_other;

// Caffe & Nvidia Digits handler : Object Recognition
void objectRecognition(const char* filename)
{

	std::stringstream command;

	command << "/home/turtlebot2/cagatay_ws/src/object_recognition/src/example.py "
	<< "/home/turtlebot2/cagatay_ws/src/object_recognition/model/snapshot_iter_399000.caffemodel "
	<< "/home/turtlebot2/cagatay_ws/src/object_recognition/model/deploy.prototxt "
	<< filename
	<< " --mean /home/turtlebot2/cagatay_ws/src/object_recognition/model/mean.binaryproto"
	<< " --labels /home/turtlebot2/cagatay_ws/src/object_recognition/model/labels.txt"
	<< " --nogpu";

	//ROS_INFO("%s \n", command.str().c_str());

	// Send the command and get result from object recognition script
	FILE *f = popen(command.str().c_str(), "r");

	char label[1024];

	while (fgets(label, sizeof(label), f) != NULL)

	{
	}

	pclose(f);

	// Prepare the message

	std_msgs::String msg;

	std::stringstream ss;

	ss << label;

	msg.data = ss.str();

	// Publish it to the space and wait for some alien to hear it
	pub.publish(msg);

	// Store the results in a txt file
	std::ofstream myfile;
	myfile.open ("/home/turtlebot2/cagatay_ws/src/object_recognition/classification_results.txt", std::ios::app);
	myfile << label << std::endl;
	myfile.close();
}

// Call back for fixation points
void fixationCallBack(const std_msgs::String::ConstPtr& msg)
{
	//ROS_INFO("Fixation Point: [%s]", msg->data.c_str());

	objectRecognition(msg->data.c_str());
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "objectrecognition");

	ros::NodeHandle n;

	ros::Subscriber sub =
	 n.subscribe("apes_robot/fixation_point",10,fixationCallBack);

	pub = n.advertise<std_msgs::String>("object_recognition/label", 10);

	ros::spin();

	return 0;
}
