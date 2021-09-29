#!/bin/bash
export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
roslaunch husky_gazebo husky_empty_world.launch


