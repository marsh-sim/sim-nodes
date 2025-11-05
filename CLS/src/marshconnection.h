#ifndef MARSHCONNECTION_H
#define MARSHCONNECTION_H

#include "marsh_config.h"
#include "mavlink/c_library_v2/mavlink_types.h"
#include <atomic>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>

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

    inline void setManagerAddress(const std::string address) { m_manager_address = address; };
    inline void setManagerPort(const std::string port) { m_manager_port = port: } ;

    void sendMessage(mavlink_message_t message);
    
  private:
    int m_marsh_socket;
    std::string m_manager_address;
    unsigned int m_manager_port;
    struct sockaddr_in m_manager_peer;
    std::atomic<bool> m_manager_connected;
    unsigned int m_system_id;
    unsigned int m_component_id;

    void sendHeartbeat(void);
    void managerTimedOut(void);

    void receiveMessage(mavlink_message_t message);
     
};


#endif // ndef MARSHCONNECTION_H
