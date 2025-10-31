#include "marshconnection.h"
#include "mavlink/c_library_v2/minimal/mavlink.h"

MarshConnection::MarshConnection(void)
: m_marsh_socket(-1),
  m_manager_address(DEFAULT_MARSH_MANAGER_ADDRESS),
  m_manager_port(DEFAULT_MARSH_MANAGER_PORT),
  m_manager_connected(false),
  m_system_id(0),
  m_component_id(0)
{
    // initialize socket for MARSH communication

  // initialize timers for heartbeat and manager
}

void MarshConnection::setManagerAddress(const std::string address)
{
  m_manager_address = address;
}

void MarshConnection::sendHeartbeat(void)
{
  mavlink_heartbeat_t heartbeat;
  heartbeat.type = MAV_TYPE_GENERIC;
  
}
