cmake_minimum_required(VERSION 3.5)

#change name of package to this
project(mpc_ros)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Python-Documentation.html

# Find dependencies this will allow CPP and python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Telem.msg"
#   "msg/CtlTraj.msg"
#  )

# # Include Cpp "include" directory
# include_directories(include)
# # Create Cpp executable
# add_executable(cpp_node src/cpp_node.cpp)
# ament_target_dependencies(cpp_node rclcpp)
# # Install Cpp executables
# install(TARGETS
#   cpp_node
#   DESTINATION lib/${PROJECT_NAME}
# )



# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/drone_node.py
  scripts/MPCTrajNode.py
  scripts/MPCPlaneNode.py
  scripts/MPCMavrosTrajNode.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

##tests 
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    tests/test_a.py
    # Add other test files here
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

#launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)