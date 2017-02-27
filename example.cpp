#include <CommProto/commproto.h>


using namespace comnet;

//@TODO fix cmake

//which packet do you want sucka???
#include <VehicleTelemetryCommand.hpp>
#include <VehicleTerminationCommand.hpp>
#include <VehicleWaypointCommand.hpp>

// Dont' know what packet to include? Include them all!!
#include <Packets.hpp>
using namespace ngcp;

//@TODO make call backs for these commands

//uart interface
#include "autopilot_interface.h"
#include "serial_port.h"

//probably redundant only add if needs be for uart interface code
/*
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
*/

// ticks.
typedef uint32_t tick_t;

tick_t tick = 0;
tick_t tick_limit = 50;

inline void Tick() {
  tick++;
  COMMS_DEBUG("Tick incremented.\n");
}

inline bool StillTicking() {
  return tick <= tick_limit;//what in the retardation
}


//uart_interface global class objects
//putting it on the heap for now because I don't want default constructor to be called
Serial_Port *serial_port;
Autopilot_Interface *autopilot_interface;


error_t VehicleWaypointCommandCallback(const comnet::Header& header, const VehicleWaypointCommand & packet, comnet::Comms& node) {

    //enable control
    autopilot_interface->enable_offboard_control();
	usleep(100);
    printf("SEND OFFBOARD COMMANDS\n");
    
    mavlink_set_position_target_local_ned_t sp;
    
    //the x y z could be wrong?
     set_position( packet.longitude, // [m] X
               packet.latitude, // [m] Y
               packet.altitude, // [m] Z
               sp         );
    //you can also set velocity and yaw look up more command if need be
    
    //apply changes    
    autopilot_interface->update_setpoint(sp);
    
    //disable control
    autopilot_interface->disable_offboard_control();
    
  return comnet::CALLBACK_SUCCESS | comnet::CALLBACK_DESTROY_PACKET;
}



int main()
{

  //CommProtocol
  Comms uav(2);
  uav.LoadKey("NGCP PROJECT 2016");
  // Configure these! port of xbee FTDI dongle and MAC address of Xbee
  uav.InitConnection(ZIGBEE_LINK, "/dev/ttyUSB0", "address", 57600);
  uav.AddAddress(1, "address");
  
  //c_uart_interface  port of FTDI/Serial which goes to pixhawk  
  
  serial_port = new Serial_Port("/dev/ttyUSB0", 57600);
  //create autopilot class with serial connection
  autopilot_interface = new Autopilot_Interface(&serial_port);
  
  

  uav.Run();

  // Replace nullptr Callbacks!!
  uav.LinkCallback(new ngcp::VehicleTelemetryCommand(),   new Callback(nullptr));
  uav.LinkCallback(new ngcp::VehicleTerminationCommand(), new Callback(nullptr));
  uav.LinkCallback(new ngcp::VehicleWaypointCommand(),    new comnet::Callback((comnet::callback_t)VehicleWaypointCommandCallback));

  while (StillTicking()) {    
    // copy current messages
	Mavlink_Messages messages = api.current_messages;
    
    //@TODO this need to be changed to send this data
    // local position in ned frame
	mavlink_local_position_ned_t pos = messages.local_position_ned;
	printf("Got message LOCAL_POSITION_NED (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
	printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

	// hires imu
	mavlink_highres_imu_t imu = messages.highres_imu;
	printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
	printf("    ap time:     %llu \n", imu.time_usec);
	printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
	printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
	printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
	printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
	printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
	printf("    temperature: %f C \n"       , imu.temperature );
    
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    Tick();
  }

  autopilot_interface->stop();
  serial_port->stop();
  delete autopilot_interfacev
  delete serial_port;
  uav.Stop();
}