cmake_minimum_required(VERSION 2.8.3)
project(youbot_inv_kine)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        trajectory_msgs
        std_msgs
        sensor_msgs
        )

find_package(Eigen3 REQUIRED COMPONENTS)

include_directories(
        INCLUDE_DIRS
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
        $ENV{EIGEN3_INCLUDE_DIR}
)

catkin_package(
        CATKIN_DEPENDS roscpp
        DEPENDS EIGEN3
)

link_directories(
        ${catkin_LIBRARY_DIRS}
)

add_executable(youbot_inv_kine
        src/main.cpp
        )

target_link_libraries(youbot_inv_kine
        ${catkin_LIBRARIES}
        )