cmake_minimum_required(VERSION 3.8)
project(total_persuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi -std=c++17)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(
  DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "visualization_msgs"
  "ackermann_msgs"
  "f1tcommon"
  ""
)

foreach(PACKAGE ${DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



add_executable(total_persuit src/total_persuit.cpp src/total_persuit_main.cpp src/spline_path.cpp src/waypoint_publisher.cpp)
ament_target_dependencies(total_persuit ${DEPENDENCIES})
#
install(TARGETS total_persuit
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

foreach(dir launch config)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()


ament_export_include_directories(include)

ament_export_dependencies(${DEPENDENCIES})



ament_package()
