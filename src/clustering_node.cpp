
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <vector>
#include <string>
#include <sstream>
#include <queue>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "Clusters.h"

using namespace std;

ros::Publisher pub;

// Clusters instance
Clusters clusters(1000.0,
"/home/turtlebot2/cagatay_ws/src/object_recognition/others_labels.txt",
5,
0.75);

// Queue for filenames
queue<string> filenames_queue;

// Utility taken from runnable.com
vector<string> split(string str, char delimiter) {
  vector<string> internal;
  stringstream ss(str); // Turn the string into a stream.
  string tok;

  while(getline(ss, tok, delimiter)) {
    internal.push_back(tok);
  }

  return internal;
}

// Caffe & Nvidia Digits handler : Object Recognition
void matchScore(const char* filename1, const char* filename2)
{
	std::stringstream command;

	command << "/home/turtlebot2/cagatay_ws/src/object_recognition/src/deepmatching_1.0.2_c++/deepmatching-static "
	<< "/home/turtlebot2/cagatay_ws/src/object_recognition/model/snapshot_iter_399000.caffemodel "
	<< "/home/turtlebot2/cagatay_ws/src/object_recognition/model/deploy.prototxt "
	<< filename1
	<< " --mean /home/turtlebot2/cagatay_ws/src/object_recognition/model/mean.binaryproto"
	<< " --labels /home/turtlebot2/cagatay_ws/src/object_recognition/model/labels.txt"
	<< " --nogpu";

	//std::system(command.str().c_str());

	// Send the command and get result from object recognition script
	FILE *f = popen(command.str().c_str(), "r");

	char label[1024];

	fgets(label, sizeof(label), f);

	pclose(f);

	// Prepare the message

	std_msgs::String msg;

	std::stringstream ss;

	ss << label;

	msg.data = ss.str();

	// Publish it to the space and wait for some alien to hear it

	pub.publish(msg);
}


// Call back for fixation points
void fixationCallBack(const std_msgs::String::ConstPtr& msg)
{

	//ROS_INFO("Fixation Point: [%s]", msg->data.c_str());

	vector<string> str_vec = split(msg->data.c_str(), ',');

	cout << str_vec[0] << " " << str_vec[1] << endl;

  // if the label is others
  if(str_vec[1] == "others")
  {
    // Get the front item from queue
    string filename = str_vec[0];

    // Run SIFT-based clustering algorithm
    clusters.runClustering(filename);
  }

  std::cout << "--------------------End---------------------" << std::endl;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "clustering");

	ros::NodeHandle n;

	ros::Subscriber sub =
	 n.subscribe("object_recognition/label",10,fixationCallBack);

  ros::spin();

  // save
  clusters.write2TXT();

	return 0;
}
