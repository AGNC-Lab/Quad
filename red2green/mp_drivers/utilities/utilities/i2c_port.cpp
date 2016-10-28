////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/i2c_port.cpp                                                                //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/i2c_port.h"
  #include <fcntl.h>
  #include <sys/ioctl.h>
  #include <unistd.h>
  using std::ostringstream;
  using std::string;
  using utilities::i2c_port;
#elif (QT_ENV)
  #include "utilities/i2c_port.h"
  using std::string;
  using utilities::i2c_port;
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  i2c_port::i2c_port(const string& name) : object("utilities","i2c_port") {
    m_name = name;

    message("i2c_port","opening port "+m_name);
    m_fd = open(m_name.c_str(),O_RDWR);

    if (m_fd == -1) {
      error("i2c_port","failed to open port "+m_name,errno);
    }
    else {
      message("i2c_port","opened port "+m_name);
    }
  }
  i2c_port::~i2c_port() {
    message("~i2c_port","closing port "+m_name);
    if (close(m_fd) == -1) {
      error("~i2c_port","failed to close port "+m_name,errno);
    }
    else {
      message("~i2c_port","closed port "+m_name);
    }
  }
  int32 i2c_port::read(uint8* p_rx, uint32 n_bytes, uint8 slave) const {
    int32 bytes_read;
    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (ioctl(m_fd,0x0703,slave) == -1) {
        error("read","ioctl failed",errno);
      }
    #else
      ioctl(m_fd,0x0703,slave);
    #endif

    bytes_read = ::read(m_fd,p_rx,n_bytes);
    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (bytes_read == -1) {
        error("read","failed to read",errno);
      }
    #endif
    return bytes_read;
  }
  int32 i2c_port::read(uint8* p_rx, uint32 n_bytes, uint8 reg, uint8 slave) const {
    int32 bytes_read;
    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (ioctl(m_fd,0x0703,slave) == -1) {
        error("read","ioctl failed",errno);
      }
    #else
      ioctl(m_fd,0x0703,slave);
    #endif

    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (::write(m_fd,&reg,1) == -1) {
        error("read","failed to write",errno);
      }
    #else
      ::write(m_fd,&reg,1);
    #endif

    bytes_read = ::read(m_fd,p_rx,n_bytes);
    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (bytes_read == -1) {
        error("read","failed to read",errno);
      }
    #endif
    return bytes_read;
  }
  int32 i2c_port::write(const uint8* p_tx, uint32 n_bytes, uint8 slave) const {
    int32 bytes_written;
    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (ioctl(m_fd,0x0703,slave) == -1) {
        error("write","ioctl failed",errno);
      }
    #else
      ioctl(m_fd,0x0703,slave);
    #endif

    bytes_written = ::write(m_fd,p_tx,n_bytes);
    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (bytes_written == -1) {
        error("write","failed to write",errno);
      }
      else if (bytes_written != (int32)n_bytes) {
        ostringstream msg;
        msg << "wrote " << bytes_written << " of " << n_bytes << " bytes";
        warning("write",msg.str());
      }
    #endif
    return bytes_written;
  }
#elif (QT_ENV)
  i2c_port::i2c_port(const string& name) : object("utilities","i2c_port") {
    UNUSED(name);
  }
  i2c_port::~i2c_port() {
    // Do nothing.
  }
  int32 i2c_port::read(uint8* p_rx, uint32 n_bytes, uint8 slave) const {
    UNUSED(p_rx);
    UNUSED(n_bytes);
    UNUSED(slave);
    return 0;
  }
  int32 i2c_port::read(uint8* p_rx, uint32 n_bytes, uint8 reg, uint8 slave) const {
    UNUSED(p_rx);
    UNUSED(n_bytes);
    UNUSED(reg);
    UNUSED(slave);
    return 0;
  }
  int32 i2c_port::write(const uint8* p_tx, uint32 n_bytes, uint8 slave) const {
    UNUSED(p_tx);
    UNUSED(n_bytes);
    UNUSED(slave);
    return 0;
  }  
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
