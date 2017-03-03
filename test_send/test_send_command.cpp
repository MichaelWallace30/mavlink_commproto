#include <CommProto/commproto.h>
using namespace comnet;
#include <VehicleWaypointCommand.hpp>

// Dont' know what packet to include? Include them all!!
#include <Packets.hpp>
using namespace ngcp;



#include <iostream>
using namespace std;



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

error_t HighResGyroCallback(const comnet::Header& header, const HighResGyro & packet, comnet::Comms& node) {

  double xGyro = packet.xgyro;
  double yGyro = packet.ygyro;
  double zGyro = packet.zgyro;
  printf("Gyro X: %f Y: %f Z: %f\n", xGyro, yGyro, zGyro);
  return comnet::CALLBACK_SUCCESS;    
}


int main()
{

  //CommProtocol
  Comms test(1);
  test.LoadKey("NGCP PROJECT 2016");
  
  
  //xbee mode
  // Configure these! port of xbee FTDI dongle and MAC address of Xbee
  //test.InitConnection(ZIGBEE_LINK, "/dev/ttyUSB0", "address", 57600);
  //test.AddAddress(1, "address");
  
  //udp mode for lcoal testing
  test.InitConnection(UDP_LINK, "1338", "127.0.0.1");
  test.AddAddress(2, "127.0.0.1", 1337);
  

  test.Run();
  //VehicleWaypointCommand(uint16_t vehicle_id = 0,float latitude = 0,float longitude = 0,float altitude = 0) 
 
/* printf("0 to send waypoint or 1 to send command mode stop: ");
  int choice;
  cin >> choice;
  if(choice == 0){
    VehicleWaypointCommand vwpc(2,1,2,3);
    test.Send(vwpc, 2);
  }else
  {
      VehicleModeCommand vmc(2,0);
      test.Send(vmc,2);
  }
  */
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  test.LinkCallback(new HighResGyro(),    new comnet::Callback((comnet::callback_t)HighResGyroCallback));
  
  while(true)
  {
	  
  }
  
  test.Stop();
}