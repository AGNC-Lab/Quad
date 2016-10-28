////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/uart_port.cpp                                                               //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/uart_port.h"
  #include <fcntl.h>
  #include <unistd.h>
  using std::ostringstream;
  using std::string;
  using utilities::uart_port;
#elif (QT_ENV)
  #include "utilities/uart_port.h"
  #include <QObject>
  using std::string;
  using utilities::uart_port;
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  uart_port::uart_port(const string& name,
                       BAUD_RATE br,
                       uint8 min_bytes,
                       uint8 timeout_ds) : object("utilities","uart_port") {
    /* Comments below pertain to the selection of min_bytes and timeout_ds, and were taken from the
    "Canonical and noncanonical mode" section from the following link: 
    http://man7.org/linux/man-pages/man3/tcsetattr.3.html */  

    /* In noncanonical mode input is available immediately (without the user
         having to type a line-delimiter character), no input processing is
         performed, and line editing is disabled.  The settings of MIN
         (c_cc[VMIN]) and TIME (c_cc[VTIME]) determine the circumstances in
         which a read(2) completes; there are four distinct cases:

         MIN == 0, TIME == 0 (polling read)
                If data is available, read(2) returns immediately, with the
                lesser of the number of bytes available, or the number of
                bytes requested.  If no data is available, read(2) returns 0.

         MIN > 0, TIME == 0 (blocking read)
                read(2) blocks until MIN bytes are available, and returns up
                to the number of bytes requested.

         MIN == 0, TIME > 0 (read with timeout)
                TIME specifies the limit for a timer in tenths of a second.
                The timer is started when read(2) is called.  read(2) returns
                either when at least one byte of data is available, or when
                the timer expires.  If the timer expires without any input
                becoming available, read(2) returns 0.  If data is already
                available at the time of the call to read(2), the call behaves
                as though the data was received immediately after the call.

         MIN > 0, TIME > 0 (read with interbyte timeout)
                TIME specifies the limit for a timer in tenths of a second.
                Once an initial byte of input becomes available, the timer is
                restarted after each further byte is received.  read(2)
                returns when any of the following conditions is met:

                *  MIN bytes have been received.

                *  The interbyte timer expires.

                *  The number of bytes requested by read(2) has been received.
                   (POSIX does not specify this termination condition, and on
                   some other implementations read(2) does not return in this
                   case.)

                Because the timer is started only after the initial byte
                becomes available, at least one byte will be read.  If data is
                already available at the time of the call to read(2), the call
                behaves as though the data was received immediately after the
                call.
    */

    m_name = name;

    memset(&m_settings,0,sizeof(m_settings)); // clear settings structure.
    m_settings.c_iflag = 0;
    m_settings.c_oflag = 0;
    m_settings.c_cflag = CS8|CREAD|CLOCAL;
    m_settings.c_lflag = 0; // set to non-canonical mode.
    m_settings.c_cc[VMIN] = min_bytes; // min number of bytes required to return.
    m_settings.c_cc[VTIME] = timeout_ds; // minimum length of time to timeout in tenths of seconds.

    message("uart_port","opening port "+m_name);
    m_fd = open(m_name.c_str(),O_RDWR);
    if (m_fd == -1) {
      error("uart_port","failed to open port "+m_name,errno);
    }
    else {
      cfsetospeed(&m_settings,br); // set output baud rate.
      cfsetispeed(&m_settings,br); // set input baud rate.
      tcsetattr(m_fd,TCSANOW,&m_settings); // set port settings to m_settings immediately. 
      message("uart_port","opened port "+m_name);
    }

    flush_input();
    flush_output();
  }
  uart_port::~uart_port() {
    message("~uart_port","closing port "+m_name);

    flush_input();
    flush_output();

    if (close(m_fd) == -1) {
      error("~uart_port","failed to close port "+m_name,errno);
    }
    else {
      message("~uart_port","closed port "+m_name);
    }
  }
  void uart_port::set_baud_rate(BAUD_RATE br) {
    cfsetospeed(&m_settings,br); // set output baud rate.
    cfsetispeed(&m_settings,br); // set input baud rate.
    tcsetattr(m_fd,TCSANOW,&m_settings); // set port settings to m_settings immediately.
  }
  void uart_port::flush_input() const {
    tcflush(m_fd,TCIFLUSH);
  }
  void uart_port::flush_output() const {
    tcflush(m_fd,TCOFLUSH);
  }
  int32 uart_port::read(uint8* p_rx, uint32 n_bytes) const {
    int32 bytes_read = ::read(m_fd,p_rx,n_bytes);
    #if !defined(SUPPRESS_RDWR_ERROR_CHECKING)
      if (bytes_read == -1) {
        error("read","failed to read",errno);
      }
    #endif
    return bytes_read;
  }
  int32 uart_port::write(const uint8* p_tx, uint32 n_bytes) const {
    int32 bytes_written = ::write(m_fd,p_tx,n_bytes);
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
  uart_port::uart_port(const string& name,
                       BAUD_RATE br,
                       uint8 min_bytes,
                       uint8 timeout_ds) : object("utilities","uart_port") {
    UNUSED(name);
    UNUSED(br);
    UNUSED(min_bytes);
    UNUSED(timeout_ds);
  }
  uart_port::~uart_port() {
    // Do nothing.
  }
  void uart_port::set_baud_rate(BAUD_RATE br) {
    UNUSED(br);
  }
  void uart_port::flush_input() const {
    // Do nothing.
  }
  void uart_port::flush_output() const {
    // Do nothing.
  }
  int32 uart_port::read(uint8* p_rx, uint32 n_bytes) const {
    UNUSED(p_rx);
    UNUSED(n_bytes);
    return 0;
  }
  int32 uart_port::write(const uint8* p_tx, uint32 n_bytes) const {
    UNUSED(p_tx);
    UNUSED(n_bytes);
    return 0;
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
