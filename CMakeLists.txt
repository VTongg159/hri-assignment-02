cmake_minimum_required(VERSION 3.16)
project(ur3_llm_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(skill_server src/skill_server.cpp)
ament_target_dependencies(skill_server
  rclcpp rclcpp_action control_msgs geometry_msgs moveit_ros_planning_interface
  moveit_msgs shape_msgs trajectory_msgs tf2_ros ros_gz_interfaces rcl_interfaces
  std_msgs visualization_msgs)

install(TARGETS skill_server DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/task_cli scripts/reset_scene DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config prompts worlds docs urdf DESTINATION share/${PROJECT_NAME})
install(FILES .env.example README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_validator test/test_validator.py)
  ament_add_pytest_test(test_student_task test/test_student_task.py)
  ament_add_pytest_test(test_executor test/test_executor.py)
  ament_add_pytest_test(test_llm_client test/test_llm_client.py)
endif()

ament_package()
