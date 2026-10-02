cmake_minimum_required(VERSION 3.8)
project(its_ros2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(its_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(Boost REQUIRED)
find_package(coal REQUIRED)


set(OpenGL_GL_PREFERENCE "GLVND")
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# === Cpp ===
# Include Cpp "include" directory
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  lib
)

# Create Cpp Libraries
## Declare a C++ library
add_library(IntrinsicTactileSensing STATIC src/IntrinsicTactileSensing.cpp)
add_library(SoftIntrinsicTactileSensing STATIC src/SoftIntrinsicTactileSensing.cpp)
#add_library(TactipGKD STATIC src/TactipGKD.cpp)

## Declare dependencies
target_link_libraries(IntrinsicTactileSensing
  coal::coal
  levmar
  lapack
  blas
)
ament_target_dependencies(IntrinsicTactileSensing rclcpp std_msgs sensor_msgs)
target_link_libraries(SoftIntrinsicTactileSensing
  IntrinsicTactileSensing
)
ament_target_dependencies(SoftIntrinsicTactileSensing rclcpp std_msgs sensor_msgs)


# Create Cpp executable
# --------------------------
# add_executable(test_thunder
#   src/test_thunder.cpp
# )
# target_link_libraries(test_thunder
#   ${PROJECT_NAME}
#   yaml-cpp
# )
# ament_target_dependencies(test_thunder rclcpp std_msgs sensor_msgs)

# --------------------------
add_executable(its_node src/its_node.cpp)
target_link_libraries(its_node
   IntrinsicTactileSensing
   coal::coal
)

add_executable(soft_its_node src/soft_its_node.cpp)
target_link_libraries(soft_its_node
  SoftIntrinsicTactileSensing
  IntrinsicTactileSensing
  coal::coal
)

add_executable(soft_its_viz src/soft_its_viz.cpp)
target_link_libraries(soft_its_viz
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
    coal::coal
    Eigen3::Eigen
)


add_executable(wrench_compensator src/wrench_compensator.cpp)
#add_executable(gkd_node src/gkd_node.cpp)
#add_executable(aruco_tracking src/aruco_tracking.cpp)


# test helper node to verify package build/run
add_executable(check_package_node tests/check_package_node.cpp)


ament_target_dependencies(its_node rclcpp std_msgs sensor_msgs its_msgs tf2 tf2_ros)
ament_target_dependencies(soft_its_node rclcpp std_msgs sensor_msgs its_msgs tf2 tf2_ros)

ament_target_dependencies(soft_its_viz rclcpp std_msgs sensor_msgs  its_msgs)


ament_target_dependencies(wrench_compensator  rclcpp  geometry_msgs  tf2  tf2_ros)

ament_target_dependencies(check_package_node rclcpp std_msgs)

install(TARGETS
  its_node
  soft_its_node
  soft_its_viz
  wrench_compensator
  check_package_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY launch config assets
  DESTINATION share/${PROJECT_NAME}
)

# ====== BUILD ======
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
