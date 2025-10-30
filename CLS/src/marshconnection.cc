#include "marshconnection.h"

MarshConnection::MarshConnection(void)
{
  setManagerAddress(m_manager_address);

  // initialize socket for MARSH communication

  // initialize timers for heartbeat and manager
}

void MarshConnection::setManagerAddress(const std::string address)
{
  // TODO
}

void MarshConnection::sendHeartbeat(void)
{
  mavlink_heartbeat_t heartbeat;
  heartbeat.type = MAV_TYPE_GENERIC;
  
}
