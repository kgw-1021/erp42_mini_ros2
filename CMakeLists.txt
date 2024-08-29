cmake_minimum_required(VERSION 3.5)
project(erp42_mini_ros2)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_mux REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)


set(
  DEPENDENCIES
  "rclcpp"
  "rclpy"
  "std_msgs"
  "ackermann_msgs"
  "ackermann_mux"
)

add_executable(ackermann_mux
  
)

ament_target_dependencies(erp42_mini_ros2 ${DEPENDENCIES})

install(
  TARGETS ackermann_mux
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

foreach(dir launch config)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

install(
  DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/ErpSerialHandler.py
  scripts/ByteHandler.py
  scripts/ErpTeleop.py
  DESTINATION include
)

set(msg_files
  msg/ErpMiniCmdMsg.msg
  msg/ErpMiniStatusMsg.msg
)



rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
