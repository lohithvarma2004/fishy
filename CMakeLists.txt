cmake_minimum_required(VERSION 3.8)
project(fish_hpurv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)  
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)      
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(
  DIRECTORY launch urdf config meshes worlds models
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.xacro"
)
add_executable(joint_publisher scripts/joint_publisher.cpp)
ament_target_dependencies(joint_publisher rclcpp trajectory_msgs)

ament_export_dependencies(ament_cmake gazebo_ros xacro rclcpp trajectory_msgs)

install(TARGETS
  joint_publisher
  DESTINATION lib/${PROJECT_NAME}
)


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
