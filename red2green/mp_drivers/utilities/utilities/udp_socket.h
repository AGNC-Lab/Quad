////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/udp_socket.h                                                                //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_UDP_SOCKET_H
#define UTILITIES_UDP_SOCKET_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/globals.h"
  #include "utilities/socket.h"
#elif (QT_ENV)
  #include "utilities/globals.h"
  #include "utilities/socket.h"
  #include <QUdpSocket>
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class udp_socket : public socket {
    private:
      #if (POSIX_ENV)
        std::string m_name;
        int m_handle;
        int m_flags;
      #elif (QT_ENV)
        std::string m_name;
        bool m_block;
        int m_flags;
        QUdpSocket m_socket;
        QUdpSocket* mp_socket;
      #endif

    public:
      udp_socket(const std::string& name, const std::string& address);
      ~udp_socket();
      void set_flags(FLAGS flags);
      void send(const uint8* p_tx, uint32 n_bytes, const std::string& dest) const;
      int32 recv(uint8* p_rx, uint32 n_bytes, std::string& src) const;
      int32 recv(uint8* p_rx, uint32 n_bytes) const;
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
