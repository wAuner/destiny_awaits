
# :fire: Team Repo destiny_awaits :fire:

this is our team repo, not the official one.
all project management is done in our meistertask board

:point_right:[Meistertask Board](https://www.meistertask.com/projects/1pvbqtkz1f/join/):point_left:

# :construction_worker: Workflow and naming conventions

Note: :exclamation: **Do not clone the repo into a Dropbox or similar folder! This can cause serious conflicts with git.** :exclamation:

Instead of forks  we are working from a central repo. Every teammember is supposed to clone the repo and work on his own branch. 
The branches will then be pushed to github and there a pull request is opened and then they will be merged into master if everything is ok. Do not merge locally and then push master.
Naming convention for branches: **[name]_[task]**

**IMPORTANT:** :exclamation: :exclamation: :exclamation: Every function or method must have a meaningful docstring which describes its purpose, arguments including argmuent type and the return value. :exclamation: :exclamation: :exclamation:

**Workflow**:

0. before you create a new branch, make sure that your master branch is up to speed by `git pull origin` while you are on your master branch
1. create a new branch for the task: `git branch [branch_name]`
2. checkout that branch: `git checkout [branch_name]` (Note: you could also combine steps 1 & 2 with `git checkout -b [branch_name]`)
3. write your code, add and commit to your branch
4. once you are finished push your branch to origin which creates a branch there with the same name: `git push -u origin [branch_name]`
5. on github create a pullrequest for this newly created branch so that it can be merged 
6. When your pullrequest was merged, you need to checkout your local `master` branch and perform a pull so that your master matches the origin: `git checkout master` followed by `git pull origin`
7. **Don't forget to write a documentation in the wiki!**

If I missed something, let me know :shipit:.

## Project structure

![](imgs/final-project-ros-graph-v2.png)

### Suggested order from classroom


1.  Waypoint Updater Node (Partial): Complete a partial waypoint updater which subscribes to `/base_waypoints` and `/current_pose` and publishes to `/final_waypoints`.
2.  DBW Node: Once your waypoint updater is publishing `/final_waypoints`, the waypoint_follower node will start publishing messages to the `/twist_cmd topic`. At this point, you have everything needed to build the dbw_node. After completing this step, the car should drive in the simulator, ignoring the traffic lights.
3. Traffic Light Detection: This can be split into 2 parts:

    3.1 Detection: Detect the traffic light and its color from the `/image_color`. The topic `/vehicle/traffic_lights` contains the exact location and status of all traffic lights in simulator, so you can test your output.
    
    3.2 Waypoint publishing: Once you have correctly identified the traffic light and determined its position, you can convert it to a waypoint index and publish it.
    
4. Waypoint Updater (Full): Use `/traffic_waypoint` to change the waypoint target velocities before publishing to `/final_waypoints`. Your car should now stop at red traffic lights and move when they are green.

### Message overview Waypoint Updater Node (Partial)
<table class="index--table--YF7cZ index--table-striped--1QFWN">
<thead>
<tr>
<th><strong>Topic</strong></th>
<th><strong>Msg Type</strong></th>
<th><strong>Notes</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td>/base_waypoints</td>
<td>styx_msgs/Lane</td>
<td>Waypoints as provided by a static .csv file.</td>
</tr>
<tr>
<td>/current_pose</td>
<td>geometry_msgs/PoseStamped</td>
<td>Current position of the vehicle, provided by the simulator or localization.</td>
</tr>
<tr>
<td>/final_waypoints</td>
<td>styx_msgs/Lane</td>
<td>This is a subset of /base_waypoints. The first waypoint is the one in /base_waypoints which is closest to the car.</td>
</tr>
</tbody>
</table>

## :blue_book: Original Readme

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

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

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

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
