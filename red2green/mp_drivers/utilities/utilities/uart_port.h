////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/uart_port.h                                                                 //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_UART_PORT_H
#define UTILITIES_UART_PORT_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/globals.h"
  #include "utilities/object.h"
  #include <termios.h>
#elif (QT_ENV)
  #include "utilities/globals.h"
  #include "utilities/object.h"
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class uart_port : public object {
    #if (POSIX_ENV)
      public:
        enum BAUD_RATE {
          BR9600=B9600,
          BR38400=B38400,
          BR115200=B115200
        };

      private:
        std::string m_name;
        struct termios m_settings;
        int m_fd;
    #elif (QT_ENV)
      public:
        enum BAUD_RATE {
          BR9600,
          BR38400,
          BR115200
        };
    #endif

    public:
      uart_port(const std::string& name, BAUD_RATE br, uint8 min_bytes=0, uint8 timeout_ds=0);
      ~uart_port();
      void set_baud_rate(BAUD_RATE br);
      void flush_input() const;
      void flush_output() const;
      int32 read(uint8* p_rx, uint32 n_bytes) const;
      int32 write(const uint8* p_tx, uint32 n_bytes) const;
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
