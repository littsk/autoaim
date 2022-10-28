cmake_minimum_required(VERSION 3.8)
project(mindvision_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_aim_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

message(STATUS "${CAMKE_SOURCE_DIR}\n")
add_executable(camera_catch_node src/camera_catch_node.cpp)

target_include_directories(camera_catch_node PRIVATE include/mindvision_camera)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(camera_catch_node PUBLIC include/mindvision_camera/x64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(camera_catch_node PUBLIC include/mindvision_camera/arm64)
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

ament_target_dependencies(camera_catch_node rclcpp sensor_msgs image_transport auto_aim_interface)

target_link_libraries(camera_catch_node libMVSDK.so ${OpenCV_LIBS})

install(TARGETS
  camera_catch_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

