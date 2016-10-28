////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/socket.h                                                                    //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_SOCKET_H
#define UTILITIES_SOCKET_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/globals.h"
  #include "utilities/object.h"
  #include <sys/socket.h>
#elif (QT_ENV)
  #ifndef MSG_DONTWAIT
    #define MSG_DONTWAIT 0x40
  #endif
  #include "utilities/globals.h"
  #include "utilities/object.h"
  #include <QObject>
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class socket : public object {
    // List of pure virtual functions:
    // (1) void set_flags(FLAGS flags)
    // (2) void send(const uint8* p_tx, uint32 n_bytes, const std::string& dest) const
    // (3) int32 recv(uint8* p_rx, uint32 n_bytes, std::string& src) const
    // (4) int32 recv(uint8* p_rx, uint32 n_bytes) const

    public:
      enum TYPE {
        IPC,
        UDP
      };
      enum FLAGS {
        NONE = 0,
        BLOCKING = 0,
        NON_BLOCKING = MSG_DONTWAIT
      };

    private:
      TYPE m_type;

    public:
      socket(const std::string& scope_name,
             const std::string& class_name,
             TYPE type) : object(scope_name,class_name) {
        m_type = type;
      }
      virtual ~socket() {
        // Do-nothing.
      }
      virtual void set_flags(FLAGS flags) = 0;
      virtual void send(const uint8* p_tx, uint32 n_bytes, const std::string& dest) const = 0;
      virtual int32 recv(uint8* p_rx, uint32 n_bytes, std::string& src) const = 0;
      virtual int32 recv(uint8* p_rx, uint32 n_bytes) const = 0;
      TYPE type() const {
        return m_type;
      }
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
