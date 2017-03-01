#
# Test main for testing.
#
cmake_minimum_required( VERSION 2.8.11 FATAL_ERROR )
project(mavlink_commproto)

# Build the library with this application, otherwise we would need to make the 
# library for them...
add_subdirectory(CommProtocol/)
add_subdirectory(test_send/)

include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/CommProtocol/CommProto/include/)
include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/c_library)
include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/CommProto-MessageStandards2017/packets/cpp/)


# Add the exe
add_executable(mavlink_commproto
 ${CMAKE_CURRENT_SOURCE_DIR}/example.cpp
 autopilot_interface.cpp
 autopilot_interface.h
 serial_port.h
 serial_port.cpp
)

target_link_libraries(mavlink_commproto CommProtocol-Static)
target_link_libraries(mavlink_commproto cryptopp-static)
target_link_libraries(mavlink_commproto libxbee3)

 #Mario? this is probably not right
if (CMAKE_VERSION VERSION_GREATER 2.8.11)  
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -pthread")
else()
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -std=c++11 -pthread")
endif()

# Property to set for C++ 11 standard.
set_property(TARGET mavlink_commproto PROPERTY CXX_STANDARD 11)


#need help Mario?
# Given that linux is different from windows
#target_link_libraries(CommProtoXBee ${COMMPROTO_FINAL_STATIC})

