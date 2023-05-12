# SPDX-FileCopyrightText: ©2023 Giordano Scarso
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.5)
project(gazebo_ros_batteries)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
  find_package(ament_cmake REQUIRED)
  find_package(gazebo_dev REQUIRED)
  find_package(gazebo_ros REQUIRED)
  find_package(gazebo_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(Boost REQUIRED COMPONENTS thread)
  find_package(rosidl_default_generators REQUIRED)
  find_package(battery_services REQUIRED)

  include_directories(include ${Boost_INCLUDE_DIRS})
  link_directories(${gazebo_dev_LIBRARY_DIRS})
  ament_export_libraries()

###########
## Build ##
###########

add_library(gazebo_ros_battery SHARED src/gazebo_ros_battery.cpp)
target_include_directories(gazebo_ros_battery PUBLIC include)
ament_target_dependencies(gazebo_ros_battery
  "gazebo_ros"
  sensor_msgs
  battery_services
  )
ament_export_libraries(gazebo_ros_battery)

target_link_libraries(gazebo_ros_battery ${Boost_LIBRARIES})
#add_dependencies(gazebo_ros_battery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION include/gazebo_ros_batteries
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
ament_export_include_directories(include)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_srvs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(rosconsole)
ament_export_dependencies(std_msgs)

install(TARGETS
  gazebo_ros_battery
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  #RUNTIME DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
  )

install(DIRECTORY xacro
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
