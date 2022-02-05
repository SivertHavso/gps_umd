cmake_minimum_required(VERSION 3.10)
project(gps_tools)

# Use C++14 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(nav_to_odom utm_navsatfix_to_odometry)
set(odom_to_nav utm_odometry_to_navsatfix)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

# Libraries / Components

add_library(${nav_to_odom} SHARED
  src/${nav_to_odom}_component.cpp)
target_include_directories(${nav_to_odom}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${nav_to_odom}
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(${nav_to_odom} "gps_tools::UtmOdometryComponent")
ament_target_dependencies(${nav_to_odom}
  "nav_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)

add_library(${odom_to_nav} SHARED
  src/${odom_to_nav}_component.cpp)
target_include_directories(${odom_to_nav}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${odom_to_nav}
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(${odom_to_nav} "gps_tools::UtmOdometryToNavSatFixComponent")
ament_target_dependencies(${odom_to_nav}
  "nav_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)

# Executables / nodes

add_executable(${nav_to_odom}_node
  src/${nav_to_odom}_node.cpp)
target_include_directories(${nav_to_odom}_node
  PUBLIC
  ${libgps_INCLUDE_DIRS}
)
target_link_libraries(${nav_to_odom}_node
  ${nav_to_odom}
)
ament_target_dependencies(${nav_to_odom}_node
  "nav_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)

add_executable(${odom_to_nav}_node
  src/${odom_to_nav}_node.cpp)
target_include_directories(${odom_to_nav}_node
  PUBLIC
  ${libgps_INCLUDE_DIRS}
)
target_link_libraries(${odom_to_nav}_node
  ${odom_to_nav}
)
ament_target_dependencies(${odom_to_nav}_node
  "nav_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)


if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
  ${nav_to_odom}
  ${odom_to_nav}
  ${nav_to_odom}_node
  ${odom_to_nav}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(PROGRAMS
    scripts/bag_converter
    nodes/fix_translator
  DESTINATION bin
)

# Export package dependencies
ament_export_dependencies(ament_cmake
  ament_cmake_python
  rclcpp
  rclpy
)
ament_export_include_directories(include)
ament_export_libraries(${nav_to_odom})
ament_export_libraries(${odom_to_nav})

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
endif()

ament_package()
