cmake_minimum_required(VERSION 3.8)
project(indefinitely_loop)

find_package(autoware_cmake REQUIRED)

autoware_package()

# find dependencies
find_package(ament_cmake REQUIRED)

ament_auto_add_executable(${PROJECT_NAME}
  src/indefinitely_loop.cpp
)

ament_auto_package()
