cmake_minimum_required(VERSION 3.8)
project(auv_pathplanning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(ament_cmake REQUIRED)
# find_package(ament_lint_auto REQUIRED)
# ament_lint_auto_find_test_dependencies()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
  
install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME})
ament_auto_add_executable(path_planning  src/path_planning.cpp)
target_include_directories(path_planning PUBLIC include)

ament_auto_package()