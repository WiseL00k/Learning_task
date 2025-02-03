cmake_minimum_required(VERSION 3.0.2)
project(buff_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        std_msgs
        pluginlib
        control_toolbox
        realtime_tools
        roslint
        hardware_interface
        controller_interface
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES buff_controller
        CATKIN_DEPENDS roscpp std_msgs control_toolbox
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/buff_controller.cpp
# )

## Declare cpp executables
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}.cpp
)

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)
