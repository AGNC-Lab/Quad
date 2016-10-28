////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/ipc_socket.h                                                                //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_IPC_SOCKET_H
#define UTILITIES_IPC_SOCKET_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"
#include "utilities/socket.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class ipc_socket : public socket {
    private:
      std::string m_name;
      std::string m_path;
      int m_handle;
      int m_flags;

    public:
      ipc_socket(const std::string& name, const std::string& address);
      ~ipc_socket();
      void set_flags(FLAGS flags);
      void send(const uint8* p_tx, uint32 n_bytes, const std::string& dest) const;
      int32 recv(uint8* p_rx, uint32 n_bytes, std::string& src) const;
      int32 recv(uint8* p_rx, uint32 n_bytes) const;
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
