cmake_minimum_required(VERSION 3.8)
project(octomap_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------
# Dependencies
# ------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)


# ------------------------
# Executables
# ------------------------

add_executable(octomap_merger_node
  src/octomap_merger_node.cpp
)

ament_target_dependencies(octomap_merger_node
  rclcpp
  octomap
  octomap_msgs
)

target_link_libraries(octomap_merger_node
  ${OCTOMAP_LIBRARIES}
)


# ------------------------
# Installation
# ------------------------
install(TARGETS
  octomap_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

# Shareable resources (launch/worlds/config)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ------------------------
# Tests / Lint
# ------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
