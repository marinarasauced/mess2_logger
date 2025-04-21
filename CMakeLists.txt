cmake_minimum_required(VERSION 3.8)
project(mess2_logger)

cmake_policy(SET CMP0115 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(rclcpp REQUIRED)

include_directories(include)

pybind11_add_module(msg_logger src/msg_logger.cpp)
pybind11_add_module(msg_parser src/msg_parser.cpp)
target_link_libraries(msg_parser PRIVATE ament_index_cpp::ament_index_cpp)

# pybind11_add_module(node_wrapper src/node_wrapper.cpp)
# target_link_libraries(node_wrapper PRIVATE ament_index_cpp::ament_index_cpp rclcpp::rclcpp)

install(TARGETS msg_logger msg_parser
  LIBRARY DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}
)

install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# ament_python_install_packages(${PROJECT_NAME})

install(PROGRAMS src/main.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
