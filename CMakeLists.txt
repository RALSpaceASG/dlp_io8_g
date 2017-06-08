cmake_minimum_required(VERSION 2.8.3)
project(dlp_io8_g)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	roslaunch
)

catkin_python_setup()

add_message_files(
	FILES
 	dlp_io8_msg.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  	CATKIN_DEPENDS message_runtime
)



