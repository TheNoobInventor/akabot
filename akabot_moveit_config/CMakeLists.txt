cmake_minimum_required(VERSION 3.22)
project(akabot_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(
  DIRECTORY config launch rviz srdf urdf 
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE
)

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
