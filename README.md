# ros-node-object-recognition

ROS package for getting output from pre-trained caffe models on NVIDIA Digits. 

To use the classification module, download your model from Digits server and extract it. Note that, some strings must be set properly before compilation. 
### Dependencies

* [OpenCV]
* [Robot Operating System] 

### Installation

Put it in your src file of your workspace and compile it with catkin_make. 
The codes are successfully compiled in Ubuntu 12.04 + ROS Hydro by using Catkin. 

### Nodes 

The package contains two nodes:

* classification node: Using python scripts written by NVIDIA to get the output trained with NVIDIA Digits 
* clustering node: Clusters the images according to their 2D Hue-Saturation Histogram. If you want to use this, your dataset must include a "others" label meaning that module cannot label the image.

### Topics 
- "apes_robot/fixation_point": std_msgs::String - The fullpath and name of image that will be labeled
- "object_recognition/label": std_msgs::String - Top 1 label of current input image example: "/home/awesomeuser/input.jpg,table"

License
----

MIT
  
