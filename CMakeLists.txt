cmake_minimum_required(VERSION 3.8)
project(mess2_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

include_directories(include)

pybind11_add_module(msg_parser src/msg_parser)
target_link_libraries(msg_parser PRIVATE ament_index_cpp::ament_index_cpp)

install(TARGETS msg_parser
  LIBRARY DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}
)

install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
