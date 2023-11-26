cmake_minimum_required(VERSION 3.8)
project(tbot_walker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

## walker_lib can now be linked to any test_exec or runtime_exec possible
add_library(walker_lib libs/walker.cpp)
target_include_directories(walker_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(walker_lib  rclcpp std_msgs geometry_msgs sensor_msgs)
ament_export_targets(walker_lib HAS_LIBRARY_TARGET)
# install the include directory to the install/include/
install(
  DIRECTORY include
  DESTINATION include
)
install(
  TARGETS walker_lib
  EXPORT walker_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

## actual walker node which links against walker_lib
add_executable(walker src/main.cpp)
target_include_directories(walker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(walker walker_lib)
ament_target_dependencies(walker rclcpp std_msgs geometry_msgs sensor_msgs)
install(TARGETS walker
  DESTINATION lib/${PROJECT_NAME})

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

install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
