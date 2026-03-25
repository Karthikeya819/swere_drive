cmake_minimum_required(VERSION 3.8)
project(swere_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(GAZEBO_PLUGIN_PATH ${CMAKE_CURRENT_SOURCE_DIR}/gazebo_plugins)

install(
  DIRECTORY
  launch 
  urdf
  DESTINATION share/${PROJECT_NAME}
)

add_library(ackerman_drive_plugin SHARED
    src/ackerman_drive_plugin.cpp
)
target_include_directories(ackerman_drive_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ackerman_drive_plugin
    "gazebo_ros" geometry_msgs nav_msgs tf2 tf2_geometry_msgs
)
ament_export_libraries(ackerman_drive_plugin)
install(TARGETS
    ackerman_drive_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

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
