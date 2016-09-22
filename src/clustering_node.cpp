
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

// Clusters instance
Clusters clusters(1000.0,
"/home/turtlebot2/cagatay_ws/src/object_recognition/others_labels.txt",
5,
0.90);

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
