cmake_minimum_required(VERSION 2.8.3)
project(ethercat_master)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  motorcortex_msgs
  roscpp
  std_msgs
  roslib
)

catkin_package(
  INCLUDE_DIRS src/control src/logic 
  CATKIN_DEPENDS roscpp message_runtime roslib
)

include_directories(
  src/control
  src/logic
  ${catkin_INCLUDE_DIRS}
)

#add_subdirectory(control)
#add_subdirectory(logic)

add_executable(ethercat_master_server src/main.cpp src/control/MainControlLoop.cpp src/control/Drive.cpp
        src/logic/Cia402FsmBase.cpp src/logic/Cia402FsmImpl.cpp src/logic/Cia402FsmTransition.cpp
        src/control/DigitalIO.h src/control/DigitalIO.cpp src/control/DigitalIO.cpp)
target_compile_options(ethercat_master_server PUBLIC -DPB_FIELD_32BIT -DLOG_LEVEL_INFO -DLOG_LEVEL_SM -DENABLE_ASSERT)
target_link_libraries(ethercat_master_server mcx-core ${catkin_LIBRARIES})

#install(TARGETS ethercat_master_server DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
#install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_PREFIX}/share/ethercat_master_server)