#include "marshconnection.h"
#include "debug.h"
#include "mavlink/c_library_v2/minimal/mavlink.h"
#include "mavlink/c_library_v2/marsh/marsh.h"
#include "mavlink_helpers.h"
#include "mavlink_types.h"
#include "marsh_config.h"
#include "minimal/mavlink_msg_heartbeat.h"
#include <netinet/in.h>
#include <sys/socket.h>
#include "timer.h"

MarshConnection::MarshConnection(void)
: m_marsh_socket(-1),
  m_manager_address(DEFAULT_MARSH_MANAGER_ADDRESS),
  m_manager_port(DEFAULT_MARSH_MANAGER_PORT),
  m_manager_connected(false),
  m_system_id(0),
  m_component_id(0)
{
  // initialize UDP socket
  m_manager_peer.sin_family = AF_INET;
  m_manager_peer.sin_port = htons(m_manager_port);
  m_manager_peer.sin_addr.s_addr = inet_addr(m_manager_address.c_str());

  m_marsh_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
     
  // initialize timers for heartbeat and manager
  Timer heartbeatTimer;
  heartbeatTimer.setmsInterval(DEFAULT_MARSH_HEARTBEAT_INTERVAL_MS);
  heartbeatTimer.setCallback([this](){ this->sendHeartbeat(); });

  Timer sendControlsTimer;
  sendControlsTimer.setmsInterval(DEFAULT_MARSH_CONTROL_LOADING_INTERVAL_MS);
  sendControlsTimer.setCallback([this](){ this->sendControlLoadingMessage(); });
  
}

void MarshConnection::sendHeartbeat(void)
{
  mavlink_heartbeat_t heartbeat;
  heartbeat.type = MARSH_TYPE_CONTROL_LOADING;
  heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
  heartbeat.base_mode = 0; // no flags applicable
  heartbeat.custom_mode = 0;
  heartbeat.system_status = MAV_STATE_ACTIVE;

  mavlink_message_t message;
  mavlink_msg_heartbeat_encode_chan(m_system_id,
                                    m_component_id,
                                    MAVLINK_COMM_0,
                                    &message,
                                    &heartbeat);

  sendMessage(message);
}

void MarshConnection::managerTimedOut(void)
{
   m_manager_connected = false;

   // FIXME: do we need this, without Qt?
   // managerConnectedChanged(m_manager_connected);
}

void MarshConnection::sendMessage(mavlink_message_t message)
{
  uint8_t send_buffer[MAVLINK_MAX_PACKET_LEN];
  const auto send_buffer_len = mavlink_msg_to_send_buffer(send_buffer, &message);
  const auto sent = sendto(m_marsh_socket, send_buffer, send_buffer_len, 0, (struct sockaddr*)&m_manager_peer, sizeof(m_manager_peer));

  if (sent < 0) {
    DEBUG_CERR("Error sending MAVLink to " << m_manager_address << ":" << m_manager_port)
  }
   
}
