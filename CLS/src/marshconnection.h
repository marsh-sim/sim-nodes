#ifndef MARSHCONNECTION_H
#define MARSHCONNECTION_H

#include "marsh_config.h"
#include "mavlink/c_library_v2/mavlink_types.h"
#include <atomic>
#include <string>

class MarshConnection
{
  public:
    MarshConnection(void);
    ~MarshConnection(void);

    uint8_t systemId(void) const { return m_system_id; }
    uint8_t componentId(void) const { return m_component_id; }
    bool managerConnected(void) const { return m_manager_connected; }
    std::string managerAddress(void) const { return m_manager_address; }
    uint32_t managerPort(void) const {return m_manager_port; }

    void setManagerAddress(const std::string address);
    void setManagerPort(const std::string port);

    void sendMessage(mavlink_message_t message);
    
  private:
    int m_marsh_socket;
    std::string m_manager_address;
    unsigned int m_manager_port;
    std::atomic<bool> m_manager_connected;
    unsigned int m_system_id;
    unsigned int m_component_id;

    void sendHeartbeat(void);
    void readPendingDatagrams(void);
    void managerTimedOut(void);

    void receiveMessage(mavlink_message_t message);
     
};


#endif // ndef MARSHCONNECTION_H
