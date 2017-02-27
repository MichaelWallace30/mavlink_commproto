#include <CommProto/commproto.h>
using namespace comnet;
#include <VehicleWaypointCommand.hpp>

// Dont' know what packet to include? Include them all!!
#include <Packets.hpp>
using namespace ngcp;





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
  VehicleWaypointCommand vwpc(2,1,2,3);
  test.Send(vwpc, 2);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  test.Stop();
}