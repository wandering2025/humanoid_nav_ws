cmake_minimum_required(VERSION 3.16)
project(t1_demo_rviz LANGUAGES CXX)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
add_executable(t1_demo_rviz main.cpp)
target_compile_features(t1_demo_rviz PRIVATE cxx_std_17)
target_compile_options(t1_demo_rviz PRIVATE -Wall -Wextra -Werror)
# Propagate the build's frozen ROS/OGRE library paths to transitive dependencies.
# This affects this local executable only, never system loader configuration.
target_link_options(t1_demo_rviz PRIVATE -Wl,--disable-new-dtags)
target_link_libraries(t1_demo_rviz PRIVATE rviz_common::rviz_common rclcpp::rclcpp Qt5::Widgets)
