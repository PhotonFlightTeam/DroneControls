cmake_minimum_required(VERSION 3.20)
project(photon_flight_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_subscriber src/camera.cpp)

target_link_libraries(camera_subscriber
 rclcpp::rclcpp 
 std_msgs::std_msgs)
target_include_directories(camera_subscriber PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_subscriber ${OpenCV_LIBRARIES})

install(TARGETS camera_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
