cmake_minimum_required(VERSION 3.10)
project(gpsd_client)

# Use C++14 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(gps_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libgps REQUIRED libgps)

add_library(${PROJECT_NAME} SHARED
  src/client.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${libgps_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(${PROJECT_NAME} "gpsd_client::GPSDClientComponent")
target_link_libraries(${PROJECT_NAME}
  ${libgps_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  "gps_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_libraries(${PROJECT_NAME})

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
