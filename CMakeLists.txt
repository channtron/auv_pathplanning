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

message(STATUS "FCL include dirs: ${FCL_INCLUDE_DIRS}")
message(STATUS "FCL libraries: ${FCL_LIBRARIES}")
message(STATUS "CCD include dirs: ${CCD_INCLUDE_DIRS}")
message(STATUS "CCD libraries: ${CCD_LIBRARIES}")

find_package(fcl REQUIRED)
find_package(CCD REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(  
  ${FCL_INCLUDE_DIRS}
  ${CCD_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

find_library(FCL_LIB fcl HINTS /usr/lib /usr/local/lib)
find_library(CCD_LIB ccd HINTS /usr/lib /usr/local/lib)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
  
install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME})

ament_auto_add_executable(path_planning  src/path_planning.cpp)
target_include_directories(path_planning PUBLIC include ${fcl_LIBRARIES} ${CCD_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(path_planning ${rclcpp_LIBRARIES} ${FCL_LIB} ${CCD_LIB})

ament_auto_add_executable(navigator  src/navigator.cpp)
target_include_directories(navigator PUBLIC include ${fcl_LIBRARIES} ${CCD_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(navigator ${rclcpp_LIBRARIES} ${FCL_LIB} ${CCD_LIB})

ament_auto_package()