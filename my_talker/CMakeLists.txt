cmake_minimum_required(VERSION 3.5)

project(my_talker)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(talker_component SHARED
  src/talker_component.cpp)
target_compile_definitions(talker_component
  PRIVATE "MY_TALKER_BUILDING_DLL")
ament_target_dependencies(talker_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(talker_component "my_talker::Talker")
set(node_plugins "${node_plugins}my_talker::Talker;$<TARGET_FILE:talker_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(main_exec
  src/main_exec.cpp)
target_link_libraries(main_exec
  talker_component)
ament_target_dependencies(main_exec
  "rclcpp")


install(TARGETS
  talker_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  main_exec
  DESTINATION lib/${PROJECT_NAME})

ament_package()
