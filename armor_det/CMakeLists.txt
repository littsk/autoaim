cmake_minimum_required(VERSION 3.5)
project(armor_det)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(TensorRT REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(image_transport REQUIRED)
find_package(auto_aim_interface REQUIRED)

add_library(tools STATIC src/tools.cpp)
target_include_directories(tools PUBLIC include)
target_include_directories(tools PRIVATE ${CUDA_INCLUDE_DIRS})

add_executable(armor_det_node src/armor_det_node.cpp)
ament_target_dependencies(armor_det_node rclcpp std_msgs image_transport sensor_msgs auto_aim_interface)
target_link_libraries(armor_det_node ${OpenCV_LIBS} ${TensorRT_LIBS} tools)
target_include_directories(armor_det_node PRIVATE ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

install(TARGETS
  armor_det_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
