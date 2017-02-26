#include <CommProto/commproto.h>


using namespace comnet;

//@TODO fix cmake

//which packet do you want sucka???
#include <VehicleTelemetryCommand.hpp>
#include <VehicleTerminationCommand.hpp>
#include <VehicleWaypointCommand.hpp>

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

//remove this calling if found just destruct heap data
//make sure we clean up though
//void quit_handler( int sig );

//not using this either
//parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);


//@TODO need new function to handle each type of commands
//commands(autopilot_interface);

//@TODO need function which reads data from mavlink on intervals
//???? look at commands example

//@TODO need function to init serial connection for nuc to pixhawk

//init uart function
//some name
/*
	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"/dev/ttyUSB0";
#endif
	int baudrate = 57600;

    /////////////////////////////
    //Need to redo
    /////////////////////////////    
	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);


	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

	
	  Instantiate a serial port object
	 
	  //This object handles the opening and closing of the offboard computer's
	  //serial port over which it will communicate to an autopilot.  It has
	  //methods to read and write a mavlink_message_t object.  To help with read
	  //and write in the context of pthreading, it gaurds port operations with a
	  //pthread mutex lock.
	
	 
	Serial_Port serial_port(uart_name, baudrate);


	
      //Instantiate an autopilot interface object
	 
	  //This starts two threads for read and write over MAVlink. The read thread
	  //listens for any MAVlink message and pushes it to the current_messages
	  //attribute.  The write thread at the moment only streams a position target
	  //in the local NED frame (mavlink_set_position_target_local_ned_t), which
	  //is changed by using the method update_setpoint().  Sending these messages
	  //are only half the requirement to get response from the autopilot, a signal
	  //to enter "offboard_control" mode is sent by using the enable_offboard_control()
	  //method.  Signal the exit of this mode with disable_offboard_control().  It's
	  //important that one way or another this program signals offboard mode exit,
	  //otherwise the vehicle will go into failsafe.
	 
	Autopilot_Interface autopilot_interface(&serial_port);

	
	  //Setup interrupt signal handler
	 
	  //Responds to early exits signaled with Ctrl-C.  The handler will command
	  //to exit offboard mode if required, and close threads and the port.
	  //The handler in this example needs references to the above objects.
	 
	 
	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	
	  //Start the port and autopilot_interface
	 // This is where the port is opened, and read and write threads are started.
	 
	serial_port.start();
	autopilot_interface.start();


	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	
	//Now we can implement the algorithm we want on top of the autopilot interface
	 
	commands(autopilot_interface);


	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	
	// Now that we are done we can stop the threads and close the port
	 
	autopilot_interface.stop();
	serial_port.stop();


	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}
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
  return tick <= tick_limit;
}


//@TODO make main loop such as xbee_test.cc example code
int main()
{
  Comms uav(2);
  uav.LoadKey("ngcp calpoly2017");

  // Configure these!
  uav.InitConnection(ZIGBEE_LINK, "COM#", "address", 1000);
  uav.AddAddress(1, "address");


  uav.Run();

  // Replace nullptr Callbacks!!
  uav.LinkCallback(new ngcp::VehicleTelemetryCommand(),   new Callback(nullptr));
  uav.LinkCallback(new ngcp::VehicleTerminationCommand(), new Callback(nullptr));
  uav.LinkCallback(new ngcp::VehicleWaypointCommand(),    new Callback(nullptr));

  while (StillTicking()) {    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    Tick();
  }

  uav.Stop();
}