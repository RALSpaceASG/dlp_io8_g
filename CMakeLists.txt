cmake_minimum_required(VERSION 2.8.3)
project(dlp_io8_g)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	message_generation
	roslaunch
)

add_message_files(FILES dlp_io8_msg.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

install(PROGRAMS scripts/dlp_io8_g_node.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
