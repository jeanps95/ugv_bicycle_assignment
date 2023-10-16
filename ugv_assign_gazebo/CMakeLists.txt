cmake_minimum_required(VERSION 2.8.3)
project(ugv_assign_gazebo)

find_package(catkin REQUIRED roslaunch)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
