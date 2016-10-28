////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/ipc_socket.cpp                                                              //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/ipc_socket.h"
  #include <fcntl.h>
  #include <sys/socket.h>
  #include <sys/un.h>
  #include <unistd.h>
  using std::string;
  using utilities::ipc_socket;
#elif (QT_ENV)
  #include "utilities/ipc_socket.h"
  using std::string;
  using utilities::ipc_socket;
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  ipc_socket::ipc_socket(const string& name, const string& address) :
    utilities::socket("utilities","ipc_socket",IPC) {
    m_name = name;
    m_path = address;
    m_flags = 0;

    // Open a socket for inter-process communication (AF_UNIX).
    m_handle = ::socket(AF_UNIX,SOCK_DGRAM,0);

    // Check if socket was created successfully.
    if (m_handle == -1) {
      error("ipc_socket","failed to create "+m_name+" socket",errno);
    }
    else {
      message("ipc_socket","created "+m_name+" socket");
    }

    // Populate address structure.
    struct sockaddr_un addr;
    memset(&addr,0,sizeof(struct sockaddr_un));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path,m_path.c_str());

    // Bind socket to address.
    if (bind(m_handle,(struct sockaddr*)&addr,sizeof(struct sockaddr_un)) == -1) {
      // If bind fails, attempt to unlink path.
      unlink(m_path.c_str());

      // Attempt to bind again.
      if (bind(m_handle,(struct sockaddr*)&addr,sizeof(struct sockaddr_un)) == -1) {
        error("ipc_socket","bind failed on "+m_name+" socket",errno);
      }
      else {
        message("ipc_socket","bind succeeded on "+m_name+" socket");
      }

    }
    else {
      message("ipc_socket","bind succeeded on "+m_name+" socket");
    }
  }
  ipc_socket::~ipc_socket() {
    message("~ipc_socket","closing "+m_name+" socket");
    if (close(m_handle) == -1) {
      error("~ipc_socket","failed to close "+m_name+" socket",errno);
    }
    else {
      unlink(m_path.c_str());
      message("~ipc_socket","closed "+m_name+" socket");
    }
  }
  void ipc_socket::set_flags(FLAGS flags) {
    m_flags = flags;
  }
  void ipc_socket::send(const uint8* p_tx, uint32 n_bytes, const string& dest) const {
    // Initialize destination address structure.
    struct sockaddr_un destination;
    memset(&destination,0,sizeof(struct sockaddr_un));
    destination.sun_family = AF_UNIX;
    strcpy(destination.sun_path,dest.c_str());

    // Send data to destination address.
    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (sendto(m_handle,p_tx,n_bytes,0,
          (struct sockaddr*)&destination,
          sizeof(struct sockaddr_un)) == -1) {
        error("send","failed to send data",errno);
      }
    #else
      sendto(m_handle,p_tx,n_bytes,0,(struct sockaddr*)&destination,sizeof(struct sockaddr_un));
    #endif
  }
  int32 ipc_socket::recv(uint8* p_rx, uint32 n_bytes, string& src) const {
    // Initialize source address structure and its length.
    struct sockaddr_un source;
    socklen_t addrlen = sizeof(struct sockaddr_un);
    memset(&source,0,addrlen);

    // Receive data from source. If source address was larger than what was allocated above,
    // then addrlen will be changed by recvfrom().
    int32 bytes_read = recvfrom(m_handle,p_rx,n_bytes,m_flags,(struct sockaddr*)&source,&addrlen);

    if ((bytes_read == -1)&&((errno == EAGAIN)||(errno == EWOULDBLOCK))) {
      bytes_read = 0;
    }

    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (bytes_read == -1) {
        error("recv","failed to receive data",errno);
      }
    #endif

    // Store source address.
    src = source.sun_path;

    return bytes_read;
  }
  int32 ipc_socket::recv(uint8* p_rx, uint32 n_bytes) const {
    // Initialize source address structure and its length.
    struct sockaddr_un source;
    socklen_t addrlen = sizeof(struct sockaddr_un);
    memset(&source,0,addrlen);

    // Receive data from source. If source address was larger than what was allocated above,
    // then addrlen will be changed by recvfrom().
    int32 bytes_read = recvfrom(m_handle,p_rx,n_bytes,m_flags,(struct sockaddr*)&source,&addrlen);

    if ((bytes_read == -1)&&((errno == EAGAIN)||(errno == EWOULDBLOCK))) {
      bytes_read = 0;
    }

    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (bytes_read == -1) {
        error("recv","failed to receive data",errno);
      }
    #endif

    return bytes_read;
  }
#elif (QT_ENV)
  ipc_socket::ipc_socket(const string& name, const string& address) :
    socket("utilities","ipc_socket",IPC) {
    // Do-nothing.
    UNUSED(name)
    UNUSED(address)
  }
  ipc_socket::~ipc_socket() {
    // Do-nothing.
  }
  void ipc_socket::set_flags(FLAGS flags) {
    // Do-nothing.
    UNUSED(flags)
  }
  void ipc_socket::send(const uint8* p_tx, uint32 n_bytes, const string& dest) const {
    // Do-nothing.
    UNUSED(p_tx)
    UNUSED(n_bytes)
    UNUSED(dest)
  }
  int32 ipc_socket::recv(uint8* p_rx, uint32 n_bytes, string& src) const {
    // Do-nothing.
    UNUSED(p_rx)
    UNUSED(n_bytes)
    UNUSED(src)
    return 0;
  }
  int32 ipc_socket::recv(uint8* p_rx, uint32 n_bytes) const {
    // Do-nothing.
    UNUSED(p_rx)
    UNUSED(n_bytes)
    return 0;
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
