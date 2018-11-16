cmake_minimum_required(VERSION 2.8.3)
project(motorcortex_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DriveIn.msg
  DriveInList.msg
  DriveOut.msg
  DriveOutList.msg
  DigitalInputs.msg
  DigitalInputsList.msg
  DigitalOutputs.msg
  DigitalOutputsList.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages()

catkin_package(
   CATKIN_DEPENDS message_runtime
)
