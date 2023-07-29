cmake_minimum_required(VERSION 3.8)
project(more_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  
  "msg/Trials1.msg"
  "srv/Alert.srv"
  "srv/Arduino1.srv"
  "srv/Comms.srv"
  "srv/Error.srv"
  "srv/Mission.srv"
  "srv/String.srv"
  "srv/Trigger.srv"
  "srv/Timer.srv"
  
)


ament_package()
