cmake_minimum_required(VERSION 3.8)
project(coverage_frontier_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/QueryNearestUnseen.srv"
  "srv/GetRoomStatus.srv"
  "srv/ReserveDoor.srv"
)

ament_package()
