cmake_minimum_required(VERSION 2.8.3)
project(receive_command)

find_package(catkin REQUIRED COMPONENTS

  message_generation
  roscpp
  serial
  std_msgs
)

add_service_files(
  FILES
  ArmService.srv
  KillService.srv  
  PosCtrlService.srv
  DockService.srv
  HoverService.srv
  ApproachService.srv
  ManipulatorService.srv
)


generate_messages(DEPENDENCIES std_msgs)


catkin_package(
  CATKIN_DEPENDS
    
    message_runtime
    serial
    std_msgs	
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(send_command_node src/send_command.cpp)

add_dependencies(send_command_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(send_command_node
  ${catkin_LIBRARIES}
)




