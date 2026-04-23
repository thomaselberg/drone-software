cmake_minimum_required(VERSION 3.5)
project(thyra)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Install mission scripts
install(DIRECTORY missions/
  DESTINATION lib/${PROJECT_NAME}/missions
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)

add_library(thyra_lib
  src/utils.cpp
)
target_include_directories(thyra_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(thyra_lib 
  Eigen3::Eigen
)

ament_target_dependencies(thyra_lib
  interfaces
  geometry_msgs
)


add_executable(thyra_mission_executor 
  src/main.cpp
)
target_link_libraries(thyra_mission_executor 
  thyra_lib
  Eigen3::Eigen

)

target_include_directories(thyra_mission_executor PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}  
)

# 
install(PROGRAMS
  script/gimbal.py
  script/mission_zero.py
  script/synthetic_camera_sim.py
  script/synthetic_cam_comparison.py
  script/aruco_detector_comparison.py
  script/mission_comparison.py
  script/wind_gust_generator.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_target_dependencies(thyra_mission_executor
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  interfaces
  cv_bridge
)

install(TARGETS
  thyra_lib
  thyra_mission_executor
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
