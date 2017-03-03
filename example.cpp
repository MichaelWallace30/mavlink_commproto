#include <CommProto/commproto.h>


using namespace comnet;

//@TODO fix cmake

//which packet do you want sucka???
#include <VehicleTerminationCommand.hpp>
#include <VehicleWaypointCommand.hpp>
#include <VehicleAttitude.hpp>
#include <VehicleBodySensedState.hpp>
#include <VehicleGlobalPosition.hpp>
#include <VehicleIdentification.hpp>
#include <VehicleTelemetryCommand.hpp>


// Dont' know what packet to include? Include them all!!
#include <Packets.hpp>
using namespace ngcp;

//uart interface
#include "autopilot_interface.h"
#include "serial_port.h"


struct HeartBeat : INHERITS_ABSPACKET {
  HeartBeat(
    uint8_t autopilot = 0,
    uint8_t base_mode = 0,
    uint32_t custom_mode = 0,
    uint8_t mavlink_version = 0,
    uint8_t system_status = 0,
    uint8_t type = 0) :
    autopilot(autopilot)
    , base_mode(base_mode)
    , custom_mode(custom_mode)
    , mavlink_version(mavlink_version)
    , system_status(system_status)
    , type(type)
    , CHAIN_ABSPACKET(HeartBeat)
  { }

  void Pack(REF_OBJECTSTREAM obj) override {
    obj << custom_mode;
    obj << base_mode;
    obj << autopilot;
    obj << mavlink_version;
    obj << system_status;
    obj << type;
  }

  void Unpack(REF_OBJECTSTREAM obj) override {
    obj >> type;
    obj >> system_status;
    obj >> mavlink_version;
    obj >> autopilot;
    obj >> base_mode;
    obj >> custom_mode;
  }

  ABSPACKET *Create() override {
    return new HeartBeat();
  }

  uint32_t custom_mode;
  uint8_t base_mode;
  uint8_t autopilot;
  uint8_t mavlink_version;
  uint8_t system_status;
  uint8_t type;
};


// Gyroscope information to send to GCS.
struct HighResGyro : INHERITS_ABSPACKET {
  HighResGyro(
    uint64_t time_usec = 0,
    real32_t abs_pressure = 0.0f,
    real32_t diff_pressure = 0.0f,
    real32_t temperature = 0.0f,
    real32_t pressure_alt = 0.0f,
    real32_t xacc = 0.0f,
    real32_t xgyro = 0.0f,
    real32_t xmag = 0.0f,
    real32_t yacc = 0.0f,
    real32_t ygyro = 0.0f,
    real32_t ymag = 0.0f,
    real32_t zacc = 0.0f,
    real32_t zgyro = 0.0f,
    real32_t zmag = 0.0f,
    uint16_t fields_updated = 0) 
    : CHAIN_ABSPACKET(HighResGyro)
    , time_usec(time_usec)
    , abs_pressure(abs_pressure)
    , diff_pressure(diff_pressure)
    , temperature(temperature)
    , pressure_alt(pressure_alt)
    , xacc(xacc)
    , xgyro(xgyro)
    , xmag(xmag)
    , yacc(yacc)
    , ygyro(ygyro)
    , ymag(ymag)
    , zacc(zacc)
    , zgyro(zgyro)
    , zmag(zmag)
  { }

  void Pack(REF_OBJECTSTREAM obj) override {
    obj << time_usec;
    obj << abs_pressure;
    obj << diff_pressure;
    obj << temperature;
    obj << pressure_alt;
    obj << xacc;
    obj << xgyro;
    obj << xmag;
    obj << yacc;
    obj << ygyro;
    obj << ymag;
    obj << zacc;
    obj << zgyro;
    obj << zmag;
    obj << fields_updated;
  }

  void Unpack(REF_OBJECTSTREAM obj) override {
    obj >> fields_updated;
    obj >> zmag;
    obj >> zgyro;
    obj >> zacc;
    obj >> ymag;
    obj >> ygyro;
    obj >> yacc;
    obj >> xmag;
    obj >> xgyro;
    obj >> xacc;
    obj >> pressure_alt;
    obj >> temperature;
    obj >> diff_pressure;
    obj >> abs_pressure;
    obj >> time_usec;
  }

  ABSPACKET *Create() override {
    return new HighResGyro();
  }

  uint64_t time_usec;
  real32_t abs_pressure;
  real32_t diff_pressure;
  real32_t temperature;
  real32_t pressure_alt;
  real32_t xacc;
  real32_t xgyro;
  real32_t xmag;
  real32_t yacc;
  real32_t ygyro;
  real32_t ymag;
  real32_t zacc;
  real32_t zgyro;
  real32_t zmag;
  uint16_t fields_updated;
};


int const GCS_NODE_ID = 1;
int const THIS_UAV_ID = 2;//change this id for you UAV *IMPORTANT

// ticks.
typedef uint32_t tick_t;

tick_t tick = 0;
bool is_ticking = true;

inline void Tick() {
  tick++;
  COMMS_DEBUG("Tick incremented.\n");
}

inline bool StillTicking() {
  return is_ticking;//what in the retardation
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


//!!!WARNING currently setting local position not GPS position WARNING!!!
/** This loop is a contorl loop that will be active under gcs control
	
	1)First check are we flying?  
		Yes: Loop this thread 
		No:  End thread
		2)Under GCS contro?
			Yes: 
				3A)Do we need to turn offboard control on?
					Yes: active control
				3B)Do we have a new waypoint?
					Yes: update waypoint
			No:
				3C)Do we need to turn offboard control off?
					Yes: turn control off
*/
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
  Comms uav(THIS_UAV_ID);
  uav.LoadKey("NGCP PROJECT 2016");
  
  
  //xbee mode
  // Configure these! port of xbee FTDI dongle and MAC address of Xbee
  //uav.InitConnection(ZIGBEE_LINK, "/dev/ttyUSB0", "address", 57600);
  //uav.AddAddress(GCS_NODE_ID, "address");
  
  //udp mode for lcoal testing
  uav.InitConnection(UDP_LINK, "1337", "127.0.0.1");
  uav.AddAddress(GCS_NODE_ID, "127.0.0.1", 1338);
  
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
        
		
  //128 position target local ned
  //256 position target clobal
  
  printf("Autopilot version capabilities: %d\n", messages.autopilot_version.capabilities);
  //get and send battery
  Battery myBatteryStatus(messages.battery_status.battery_remaining);// 0 is 0%: 100 is 100%
  uav.Send(myBatteryStatus, GCS_NODE_ID);
  
  // System status
  VehicleSystemStatus my_systemStatus;
  my_systemStatus.vehicle_id = uav.GetUniqueId(); // system id...
  my_systemStatus.vehicle_mode = under_gcsControl; // vehicle mode.
  my_systemStatus.vehicle_state = messages.sys_status.onboard_control_sensors_present; // I'm going to assume that is what vehicle state is...
  uav.Send(my_systemStatus, GCS_NODE_ID);

  //gps
  VehicleGlobalPosition my_globalPosition(
    uav.GetUniqueId(), 
    messages.global_position_int.lon, 
    messages.global_position_int.lat, 
    messages.global_position_int.alt, 
    messages.global_position_int.vx,
    messages.global_position_int.vy,
    messages.global_position_int.vz
  );
  uav.Send(my_globalPosition, GCS_NODE_ID);

  //attitude
  VehicleAttitude my_attitude(
    uav.GetUniqueId(),
    messages.attitude.roll,
    messages.attitude.pitch,
    messages.attitude.yaw
  );
  uav.Send(my_attitude, GCS_NODE_ID);//sassy

  //gyro
  HighResGyro gyro;
  gyro.fields_updated = messages.highres_imu.fields_updated;
  gyro.abs_pressure = messages.highres_imu.abs_pressure;
  gyro.diff_pressure = messages.highres_imu.diff_pressure;
  gyro.pressure_alt = messages.highres_imu.pressure_alt;
  gyro.temperature = messages.highres_imu.temperature;
  gyro.time_usec = messages.highres_imu.time_usec;
  gyro.xacc = messages.highres_imu.xacc;
  gyro.xgyro = messages.highres_imu.xgyro;
  gyro.xmag = messages.highres_imu.xmag;
  gyro.yacc = messages.highres_imu.yacc;
  gyro.ygyro = messages.highres_imu.ygyro;
  gyro.ymag = messages.highres_imu.ymag;
  gyro.zacc = messages.highres_imu.zacc;
  gyro.zgyro = messages.highres_imu.zgyro;
  gyro.zmag = messages.highres_imu.zmag;
  uav.Send(gyro, GCS_NODE_ID);

  // not sending.
  
    /* I don't think any one needs this
  VehicleIdentification my_identification(
    uav.GetUniqueId(),
    0xFF // something 8-bit that needs to identify vehicle types.
  );
  
  HeartBeat heartbeat;
  heartbeat.autopilot = messages.heartbeat.autopilot;
  heartbeat.base_mode = messages.heartbeat.base_mode;
  heartbeat.custom_mode = messages.heartbeat.custom_mode;
  heartbeat.mavlink_version = messages.heartbeat.mavlink_version;
  heartbeat.system_status = messages.heartbeat.system_status;
  heartbeat.type = messages.heartbeat.type;
  uav.Send(heartbeat, GCS_NODE_ID);
  */
  

        //how does this differ from global position?
        // Ans: I'm guessing position relative to something, maybe the displacement between this vehicle and gcs.
        //messages.local_position_ned.x;
        //messages.local_position_ned.y;
        //messages.local_position_ned.z;
	
        
        //position target??? local and global
        
        
        //@TODO this need to be changed to send this data to GCS (1)
	/*				AVAILBLE DATA INSIDE current_messages

                                                                          [X] Marks that we have implemented the send for it.
                                                                          [R] Marks that this packet is not found and required in the Default Packets folder.
                                                                          [U] Marks that this packet is unknown whether we need to send.
	int sysid;
	int compid;
	// Heartbeat
	mavlink_heartbeat_t heartbeat;                                          [R]
	// System Status
	mavlink_sys_status_t sys_status;                                        [X]
	// Battery Status
	mavlink_battery_status_t battery_status;                                [X]
	// Radio Status
	mavlink_radio_status_t radio_status;                                    [R]
	// Local Position
	mavlink_local_position_ned_t local_position_ned;                        [R]
	// Global Position
	mavlink_global_position_int_t global_position_int;                      [x]
	// Local Position Target
	mavlink_position_target_local_ned_t position_target_local_ned;          [R]
	// Global Position Target
	mavlink_position_target_global_int_t position_target_global_int;        
	// HiRes IMU
	mavlink_highres_imu_t highres_imu;                                      [R]
	// Attitude
	mavlink_attitude_t attitude;                                            [X]
	// Time Stamps
	Time_Stamps time_stamps;                                                [U]
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


//@TODO Possible commands to control waypoints and attitude
//https://pixhawk.ethz.ch/mavlink/

//@TODO need to send new commands
//checking message need to request the data I think MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
//recv mavlink_msg_autopilot_version.h and check capabilities for control

//MAV_CMD bunch of options look them up

//MAV_PROTOCOL_CAPABILITY
//       MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT 86
//		mavlink_set_position_target_global_int_t
//Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability.
//64	MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET	Autopilot supports commanding attitude offboard.
//128	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED	Autopilot supports commanding position and velocity targets in local NED frame.
//256	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT	Autopilot supports commanding position and velocity targets in global scaled integers.
