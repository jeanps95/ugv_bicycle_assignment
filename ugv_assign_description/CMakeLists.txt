cmake_minimum_required(VERSION 2.8.3)
project(ugv_assign_description)

find_package(catkin REQUIRED COMPONENTS
    roslaunch
    urdf
    xacro
)

catkin_package()

install(DIRECTORY launch urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
