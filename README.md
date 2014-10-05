IGVC2014
========

Code for the Universtiy of Texas Robotics and Automation Society's entry in IGVC 2014

Installation instructions:
cd
mkdir igvc_ros && igvc_ros ros
rosws init . /opt/ros/hydro
[Download igvc.rosinstall]
rosws merge [path to igvc.rosinstall]
rosws update

Edit your bashrc and add:
source $HOME/igvc_ros/setup.bash
export GAZEBO_MODEL_PATH=`rospack find igvc_sim`/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=`rospack find igvc_sim`/plugins:$GAZEBO_PLUGIN_PATH
