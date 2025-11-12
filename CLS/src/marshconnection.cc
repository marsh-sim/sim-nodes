#include "marshconnection.h"
#include "controlloadingdata.h"
#include "debug.h"
#include "mavlink/c_library_v2/minimal/mavlink.h"
#include "mavlink/c_library_v2/marsh/marsh.h"
#include "mavlink_helpers.h"
#include "mavlink_types.h"
#include "marsh_config.h"
#include "minimal/mavlink_msg_heartbeat.h"
#include "marsh/mavlink_msg_control_loading_axis.h"
#include <netinet/in.h>
#include <sys/socket.h>
#include <chrono>

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

  // Initialize and start heartbeat timer
  m_heartbeat_timer.setmsInterval(DEFAULT_MARSH_HEARTBEAT_INTERVAL_MS);
  m_heartbeat_timer.setCallback([this](){ this->sendHeartbeat(); });
  m_heartbeat_timer.start();

  // Initialize and start control loading timer
  m_control_loading_timer.setmsInterval(DEFAULT_MARSH_CONTROL_LOADING_INTERVAL_MS);
  m_control_loading_timer.setCallback([this](){ this->sendControlLoadingMessage(); });
  m_control_loading_timer.start();

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

void MarshConnection::sendControlLoadingMessage(void)
{
  // Get timestamp in milliseconds since epoch
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
  uint32_t time_boot_ms = static_cast<uint32_t>(millis);

  // Send control loading message for each axis
  for (size_t i = 0; i < MAX_CONTROL_AXES; i++)
  {
    // Create and populate the control loading axis message
    mavlink_control_loading_axis_t control_loading_msg;
    control_loading_msg.time_boot_ms = time_boot_ms;
    control_loading_msg.axis = g_controlLoadingData[i].axis;
    control_loading_msg.position = g_controlLoadingData[i].position;
    control_loading_msg.velocity = g_controlLoadingData[i].velocity;
    control_loading_msg.force = g_controlLoadingData[i].force;

    // Encode the message
    mavlink_message_t message;
    mavlink_msg_control_loading_axis_encode_chan(m_system_id,
                                                 m_component_id,
                                                 MAVLINK_COMM_0,
                                                 &message,
                                                 &control_loading_msg);

    // Send the message
    sendMessage(message);
  }
}
