////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/udp_socket.cpp                                                              //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/udp_socket.h"
  #include <netinet/in.h>
  #include <sys/socket.h>
  #include <unistd.h>
#elif (QT_ENV)
  #include "utilities/udp_socket.h"
  #include <QHostAddress>
#endif
using std::string;
using utilities::ipv4_to_string;
using utilities::string_to_ipv4;
using utilities::udp_socket;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  udp_socket::udp_socket(const string& name, const string& address) :
    utilities::socket("utilities","udp_socket",UDP) {
    uint32 ip;
    uint16 port;
    string_to_ipv4(address,ip,port);

    m_name = name;
    m_flags = 0;

    // Open a UDP socket with IP communication domain (AF_INET), and datagram service (SOCK_DGRAM).
    m_handle = ::socket(AF_INET,SOCK_DGRAM,0);

    // Check if socket was created successfully.
    if (m_handle == -1) {
      error("udp_socket","failed to create "+m_name+" socket",errno);
    }

    // Populate address structure.
    struct sockaddr_in addr;
    memset(&addr,0,sizeof(struct sockaddr_in));
    addr.sin_family = AF_INET;
    if (ip == 0) {
      addr.sin_addr.s_addr = htonl(INADDR_ANY);
    }
    else {
      addr.sin_addr.s_addr = htonl(ip);
    }
    addr.sin_port = htons(port);

    // Bind socket to address.
    if (bind(m_handle,(struct sockaddr*)&addr,sizeof(struct sockaddr_in)) == -1) {
      error("udp_socket","failed to bind "+m_name+" socket to "+address,errno);
    }
    else {
      message("udp_socket",m_name+" socket bound to "+address);
    }
  }
  udp_socket::~udp_socket() {
    if (close(m_handle) == -1) {
      error("~udp_socket","failed to close "+m_name+" socket",errno);
    }
    else {
      message("~udp_socket","closed "+m_name+" socket");
    }
  }
  void udp_socket::set_flags(FLAGS flags) {
    m_flags = flags;
  }
  void udp_socket::send(const uint8* p_tx, uint32 n_bytes, const string& dest) const {
    uint32 ip;
    uint16 port;
    string_to_ipv4(dest,ip,port);

    // Initialize destination address structure.
    struct sockaddr_in destination;
    memset(&destination,0,sizeof(struct sockaddr_in));
    destination.sin_family = AF_INET;
    destination.sin_addr.s_addr = htonl(ip);
    destination.sin_port = htons(port);

    // Send data to destination address.
    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (sendto(m_handle,p_tx,n_bytes,0,
          (struct sockaddr*)&destination,
          sizeof(struct sockaddr_in)) == -1) {
        error("send","failed to send data",errno);
      }
    #else
      sendto(m_handle,p_tx,n_bytes,0,(struct sockaddr*)&destination,sizeof(struct sockaddr_in));
    #endif
  }
  int32 udp_socket::recv(uint8* p_rx, uint32 n_bytes, string& src) const {
    // Initialize source address structure and its length.
    struct sockaddr_in source;
    socklen_t addrlen = sizeof(struct sockaddr_in);
    memset(&source,0,addrlen);

    // Receive data from source. If source address was larger than what was allocated above,
    // then addrlen will be changed by recvfrom().
    int32 bytes_read = recvfrom(m_handle,p_rx,n_bytes,m_flags,(struct sockaddr*)&source,&addrlen);

    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (bytes_read == -1) {
        error("recv","failed to receive data",errno);
      }
    #endif

    // Store source address.
    uint32 ip = source.sin_addr.s_addr;
    uint16 port = source.sin_port;
    src = ipv4_to_string(ip,port);

    return bytes_read;
  }
  int32 udp_socket::recv(uint8* p_rx, uint32 n_bytes) const {
    // Initialize source address structure and its length.
    struct sockaddr_in source;
    socklen_t addrlen = sizeof(struct sockaddr_in);
    memset(&source,0,addrlen);

    // Receive data from source. If source address was larger than what was allocated above,
    // then addrlen will be changed by recvfrom().
    int32 bytes_read = recvfrom(m_handle,p_rx,n_bytes,m_flags,(struct sockaddr*)&source,&addrlen);

    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (bytes_read == -1) {
        error("recv","failed to receive data",errno);
      }
    #endif

    return bytes_read;
  }
#elif (QT_ENV)
  udp_socket::udp_socket(const string& name, const string& address) :
    socket("utilities","udp_socket",UDP) {
    uint32 ip;
    uint16 port;
    string_to_ipv4(address,ip,port);

    // Store socket name and port number.
    m_name = name;
    mp_socket = &m_socket;
    set_flags(NONE);

    // Print that socket was created successfully.
    message("udp_socket","created "+m_name+" socket");

    // Bind socket to IP address and port number.
    if (mp_socket->bind(QHostAddress((quint32)ip),(quint16)port)) {
      message("udp_socket",m_name+" socket bound to "+address);
    }
    else {
      warning("udp_socket","failed to bind "+m_name+" socket to "+address);
    }
  }
  udp_socket::~udp_socket() {
    message("~udp_socket","closed "+m_name+" socket");
  }
  void udp_socket::set_flags(FLAGS flags) {
    m_flags = flags;

    if ((m_flags)&(NON_BLOCKING)) {
      m_block = false;
    }
    else {
      m_block = true;
    }
  }
  void udp_socket::send(const uint8* p_tx, uint32 n_bytes, const string& dest) const {
    uint32 ip;
    uint16 port;
    string_to_ipv4(dest,ip,port);

    mp_socket->writeDatagram((char*)p_tx,(qint16)n_bytes,QHostAddress((quint32)ip),(quint16)port);
  }
  int32 udp_socket::recv(uint8* p_rx, uint32 n_bytes, string& src) const {
    // Instantiate temporary QT ip and port variables.
    QHostAddress qt_ip;
    quint16 qt_port;

    // Block if m_block is set.
    if (m_block) {
      mp_socket->waitForReadyRead();
    }

    // Perform read operation.
    qint64 bytes_read = mp_socket->readDatagram((char*)p_rx,(qint64)n_bytes,&qt_ip,&qt_port);

    // Convert ip and port address to uin32 and uint16.
    uint32 ip = (uint32)qt_ip.toIPv4Address();
    uint16 port = (uint16)qt_port;
    src = ipv4_to_string(ip,port);

    return (int32)bytes_read;
  }
  int32 udp_socket::recv(uint8* p_rx, uint32 n_bytes) const {
    // Instantiate temporary QT ip and port variables.
    QHostAddress qt_ip;
    quint16 qt_port;

    // Block if m_block is set.
    if (m_block) {
      mp_socket->waitForReadyRead();
    }

    // Perform read operation.
    qint64 bytes_read = mp_socket->readDatagram((char*)p_rx,(qint64)n_bytes,&qt_ip,&qt_port);

    return (int32)bytes_read;
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
