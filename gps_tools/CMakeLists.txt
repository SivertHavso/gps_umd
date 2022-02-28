cmake_minimum_required(VERSION 3.10)
project(gps_tools)

# Use C++14 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(gps_to_odom utm_gpsfix_to_odometry)
set(nav_to_odom utm_navsatfix_to_odometry)
set(odom_to_nav utm_odometry_to_navsatfix)

include_directories(include)

# # Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
# find_package(GeographicLib REQUIRED COMPONENTS STATIC)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()



# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})


# utm_gpsfix_to_odometry
ament_auto_add_library(${gps_to_odom} SHARED src/${gps_to_odom}.cpp)
rclcpp_components_register_nodes(${gps_to_odom} "gps_tools::UTMGPSFixToOdometryComponent")
# target_link_libraries(
#   ${gps_to_odom}
#   GeographicLib
# )

ament_auto_add_executable(${gps_to_odom}_node src/${gps_to_odom}_node.cpp)


# # utm_navsatfix_to_odometry
# ament_auto_add_library(${nav_to_odom} SHARED src/${nav_to_odom}.cpp)
# rclcpp_components_register_nodes(${nav_to_odom} "gps_tools::UTMNavSatFixToOdometryComponent")

# ament_auto_add_executable(${nav_to_odom}_node src/${nav_to_odom}_node.cpp)

# # utm_odometry_to_navsatfix
# ament_auto_add_library(${odom_to_nav} SHARED src/${odom_to_nav}.cpp)
# rclcpp_components_register_nodes(${odom_to_nav} "gps_tools::UTMOdometryToNavSatFixComponent")

# ament_auto_add_executable(${odom_to_nav}_node src/${odom_to_nav}_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # ignore copyright checks
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_copyright
  )

  # run cpplint without copyright filter
  find_package(ament_cmake_cpplint)
  ament_cpplint(
    FILTERS -legal/copyright
  )

  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # add_subdirectory(test)
endif()

ament_auto_package()
