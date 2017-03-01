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

//uart interface
#include "autopilot_interface.h"
#include "serial_port.h"

// ticks.
typedef uint32_t tick_t;

tick_t tick = 0;
tick_t tick_limit = 500;

inline void Tick() {
  tick++;
  COMMS_DEBUG("Tick incremented.\n");
}

inline bool StillTicking() {
  return tick <= tick_limit;//what in the retardation
}


CommMutex flyingMutex;
CommMutex controlMutex;
CommMutex updateControlMutex;

bool doneFlying = false;
bool under_gcsControl = false;
bool onboard_control_active = false;//used to prevent turn control on reptivily
bool updateNewControlPosition = false;//on change position if new position recv
CommMutex coordMutex;
double xLongitude, yLatitude, zAltitude;//no race condition only write in waypoint command callback


CommThread gcs_control_thread;


/**
   Enable Control from GCS.
*/
void TakeControl(bool enable) 
{
  CommLock lock(controlMutex);
  under_gcsControl = enable;
}

/**
   Finish flying.
*/
void FinishFlying(bool enable) {
  CommLock lock(flyingMutex);
  doneFlying = enable;
}
/** 
  waypoint has changed update
*/
void UpdatedControl(bool enable){
	CommLock lock(updateControlMutex);
	updateNewControlPosition = enable;
}

//uart_interface global class objects
//putting it on the heap for now because I don't want default constructor to be called
Serial_Port *serial_port;
Autopilot_Interface *autopilot_interface;

void gcsControlThread()
{

    while(!doneFlying)//loop until program is done
    {
        
        if(under_gcsControl)//just switch control on or off depending on gcsControl boolean
        {
            if(!onboard_control_active)//enable control
            {
                printf("Turning Offboard Contorl On\n");
                autopilot_interface->enable_offboard_control();
                onboard_control_active = true;
                usleep(100);
            }
            
            if(updateNewControlPosition){
                mavlink_set_position_target_local_ned_t sp;
    
				printf("Updating new waypoint\n");
                //the x y z could be wrong?
                set_position(xLongitude, // [m] X
                        yLatitude, // [m] Y
                        zAltitude, // [m] Z
                        sp         );
                //you can also set velocity and yaw look up more command if need be
                
                //apply changes    
                autopilot_interface->update_setpoint(sp);
				UpdatedControl(false);//new way point processed
            }
        }
        else//either GCS will turn control off or position is reached and control is turned off
        {
            //are we in control?
            if(onboard_control_active){
                //disable control
				printf("Turning Offboard Contorl Off\n");
                onboard_control_active = false;
                autopilot_interface->disable_offboard_control();
            }
        }//end switch control mode
        
        //check if dest found and turn of control
        //truncating to int for accuracy reduction
        //might have to increase tollerance for fixed wing planes
        if(under_gcsControl){
            mavlink_local_position_ned_t pos;
            pos = autopilot_interface->current_messages.local_position_ned;	
            if(under_gcsControl && ( ((int)(pos.x) != (int)(xLongitude)) && 
                                ((int)(pos.y) != (int)(yLatitude)) && 
                                ((int)(pos.z) != ((int)(zAltitude))))){
                under_gcsControl = false;
                
            }
        }
            
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

error_t VehicleModeCommandCallback(const comnet::Header& header, const VehicleModeCommand & packet, comnet::Comms& node) {

  TakeControl(packet.vehicle_mode); // 0 for autonomous. Maybe enum is better?
  printf("CommandMode: %d\n", under_gcsControl);
  return comnet::CALLBACK_SUCCESS;    
}

error_t VehicleWaypointCommandCallback(const comnet::Header& header, const VehicleWaypointCommand & packet, comnet::Comms& node) {

    xLongitude = packet.longitude; // [m] X
    yLatitude = packet.latitude; // [m] Y
    zAltitude = packet.altitude;// [m] Z	
	UpdatedControl(true);//let the thread know new way point recieved  
    printf("New Position Recv: %f %f %f\n", packet.longitude, packet.latitude, packet.altitude);
  return comnet::CALLBACK_SUCCESS | comnet::CALLBACK_DESTROY_PACKET;
}



int main()
{
  // break off for GCS Simulation.
  gcs_control_thread = CommThread(gcsControlThread);

  //CommProtocol
  Comms uav(2);
  uav.LoadKey("NGCP PROJECT 2016");
  
  
  //xbee mode
  // Configure these! port of xbee FTDI dongle and MAC address of Xbee
  //uav.InitConnection(ZIGBEE_LINK, "/dev/ttyUSB0", "address", 57600);
  //uav.AddAddress(1, "address");
  
  //udp mode for lcoal testing
  uav.InitConnection(UDP_LINK, "1337", "127.0.0.1");
  uav.AddAddress(1, "127.0.0.1", 1338);
  
  //c_uart_interface  port of FTDI/Serial which goes to pixhawk    
  serial_port = new Serial_Port("/dev/ttyACM0", 57600);
  //create autopilot class with serial connection
  autopilot_interface = new Autopilot_Interface(serial_port);
  serial_port->start();
  autopilot_interface->start();

  uav.Run();

  // Replace nullptr Callbacks!!
  uav.LinkCallback(new ngcp::VehicleTelemetryCommand(),   new Callback(nullptr));
  uav.LinkCallback(new ngcp::VehicleTerminationCommand(), new Callback(nullptr));
  uav.LinkCallback(new ngcp::VehicleWaypointCommand(),    new comnet::Callback((comnet::callback_t)VehicleWaypointCommandCallback));
  uav.LinkCallback(new ngcp::VehicleModeCommand(),    new comnet::Callback((comnet::callback_t)VehicleModeCommandCallback));
  

  while (StillTicking()) {    
        
		
	// local position in ned frame
	Mavlink_Messages messages = autopilot_interface->current_messages;
	mavlink_battery_status_t my_battery_status = messages.battery_status;
	
	Battery myBatteryStatus(my_battery_status.battery_remaining);// 0 is 0%: 100 is 100%
	
    //@TODO this need to be changed to send this data to GCS (1)
	/*				AVAILBLE DATA INSIDE current_messages
	int sysid;
	int compid;
	// Heartbeat
	mavlink_heartbeat_t heartbeat;
	// System Status
	mavlink_sys_status_t sys_status;
	// Battery Status
	mavlink_battery_status_t battery_status;
	// Radio Status
	mavlink_radio_status_t radio_status;
	// Local Position
	mavlink_local_position_ned_t local_position_ned;
	// Global Position
	mavlink_global_position_int_t global_position_int;
	// Local Position Target
	mavlink_position_target_local_ned_t position_target_local_ned;
	// Global Position Target
	mavlink_position_target_global_int_t position_target_global_int;
	// HiRes IMU
	mavlink_highres_imu_t highres_imu;
	// Attitude
	mavlink_attitude_t attitude;
	// Time Stamps
	Time_Stamps time_stamps;
	*/
	
	//mavlink_local_position_ned_t pos = autopilot_interface->current_messages.local_position_ned; 	
	//printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );
	// hires imu
	//mavlink_highres_imu_t imu = messages.highres_imu;
	//printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
	//printf("    ap time:     %llu \n", imu.time_usec);
	//printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
	//printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
	//printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
	//printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
	//printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
	//printf("    temperature: %f C \n"       , imu.temperature );
    
	
	
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    Tick();
  }

  autopilot_interface->stop();
  serial_port->stop();
  delete autopilot_interface;
  delete serial_port;
  uav.Stop();

  TakeControl(false);
  FinishFlying(true);  
  usleep(200);
  gcs_control_thread.Join();
  
}
