
# Capstone Project - Team destiny_awaits 

This is our project repo for the final System Integration Project within the Self-Driving Car Engineer Nanodegree Program.

## Team Members

 * Andreas Daiminger    
 * Winfried Auner       
 * Yingxiang Ni              
 * Steven Lee           
 * Markus Penzel

## Project structure

![](imgs/final-project-ros-graph-v2.png)

### DWB Node

The DBW Node part can be split up into 3 part:

 * #### steering

   For steering we used the provided yaw_controller. To get a smoother result we implemented a low-pass filter for the velocity.

 * #### throttle

   For throttle we used the provided PID controller with empiric adjusted values. To smoother the transition between the diffrent        velocitys we used a low-pass filter.

 * #### brake

   The brake is depended with the throttle. If the throttle is beneath a certain threshold the brake value will be set. The value can be calculated using the vehicle mass, the wheel radius and the velocity difference. If the vehicle stays still, the brake will be set to a maximum value.


### Traffic Light Node

For the traffic light detection we used a faster RCNN model and trained it, with the [bosch traffic light dataset](https://hci.iwr.uni-heidelberg.de/node/6132). After training we had some issues predicting the right classes in the simulator and at the ros bag data, so we included [a labeld ros bag and a labeld simulator dataset](https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/view).

Here are some examples of the simulator track:
![](imgs/wa_sim_detec.jpg) 
![](imgs/wa_sim_detec1.jpg)


Here are examples of the bag trainingset:
![](imgs/wa_bag_detec.jpg)
![](imgs/wa_bag_detec1.jpg)

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).





### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
