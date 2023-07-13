cmake_minimum_required(VERSION 3.5)
project(onboard_controller)

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
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp  REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(GeographicLib REQUIRED)

find_package(template_msgs REQUIRED)

# Add extra custom messages here
# find_package(template_msgs REQUIRED)

# Include externa libraries, e.g. libinterpolate
add_subdirectory(external/libInterpolate)


file(GLOB HEADER_FILES include/*.hpp)
file(GLOB SRC_FILES src/*.cpp)

add_executable(controller ${SRC_FILES} ${HEADER_FILES})
ament_target_dependencies(controller
  rclcpp std_msgs std_srvs trajectory_msgs
  mavros_msgs geometry_msgs tf2 tf2_ros 
  template_msgs
  # template_msgs
)
target_include_directories(controller PRIVATE include)

# Include external libraries for the controller executable
target_link_libraries( controller libInterpolate::Interpolate )

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


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
