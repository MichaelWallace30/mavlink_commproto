#
# Test main for testing.
#
cmake_minimum_required( VERSION 2.8 )
project(mavlink_commproto)

include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/CommProtocol/CommProto/include/)
include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/c_uart_interface_example/mavlink/include/mavlink/v1.0/)
include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/c_uart_interface_example/)
include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/CommProto-MessageStandards2017/packets/cpp/)


add_executable(mavlink_commproto
 ${CMAKE_CURRENT_SOURCE_DIR}example.cpp)

 #Mario? this is probably not right
if (CMAKE_VERION VERSION_GREATER 2.8.11)  
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -pthread")
else()
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -std=c++11 -pthread")
endif()

#need help Mario?
# Given that linux is different from windows
#target_link_libraries(CommProtoXBee ${COMMPROTO_FINAL_STATIC})
#target_link_libraries(CommProtoXBee cryptopp-static)
#target_link_libraries(CommProtoXBee libxbee3)
