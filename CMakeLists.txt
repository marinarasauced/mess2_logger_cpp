cmake_minimum_required(VERSION 3.8)
project(mess2_logger_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(OpenCV REQUIRED)

find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(log_topics_to_jpgs src/log_topics_to_jpgs.cpp)

target_link_libraries(log_topics_to_jpgs ${Boost_LIBRARIES})

ament_target_dependencies(
  log_topics_to_jpgs
  "rclcpp"
  "cv_bridge"
  "sensor_msgs"
)

install(
  TARGETS log_topics_to_jpgs
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
