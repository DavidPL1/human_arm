cmake_minimum_required(VERSION 3.1.3)
project(human_arm_motion_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	actionlib_msgs
	message_generation
	std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

add_action_files(
	FILES
	PlayGesture.action
)

add_service_files(
	FILES
	SaveGesture.srv
)

generate_messages(
	DEPENDENCIES
	actionlib_msgs
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS actionlib_msgs std_msgs
)
