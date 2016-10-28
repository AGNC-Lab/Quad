////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/i2c_port.h                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_I2C_PORT_H
#define UTILITIES_I2C_PORT_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"
#include "utilities/mutex.h"
#include "utilities/object.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class i2c_port : public object {
    #if (POSIX_ENV)
      private:
        std::string m_name;
        int m_fd;
    #endif

    public:
      i2c_port(const std::string& name);
      ~i2c_port();
      int32 read(uint8* p_rx, uint32 n_bytes, uint8 slave) const;
      int32 read(uint8* p_rx, uint32 n_bytes, uint8 reg, uint8 slave) const;
      int32 write(const uint8* p_tx, uint32 n_bytes, uint8 slave) const;
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
