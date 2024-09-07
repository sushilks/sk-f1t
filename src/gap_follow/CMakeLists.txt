cmake_minimum_required(VERSION 3.8)
project(gap_follow)
set(CMAKE_VERBOSE_MAKEFILE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(ackermann_msgs REQUIRED)
# find_package(f1tcommon REQUIRED)

include_directories(include)

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
)

foreach(PACKAGE ${DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

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

add_executable(reactive_node src/gap_follow.cpp src/reactive_node.cpp)
ament_target_dependencies(reactive_node ${DEPENDENCIES})

install(TARGETS reactive_node
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
