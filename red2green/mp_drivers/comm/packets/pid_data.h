////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/pid_data.h                                                                    //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PACKETS_PID_DATA_H
#define PACKETS_PID_DATA_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/packet.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace packets {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class pid_data : public utilities::packet<comm::meta,comm::meta::ID_PID_DATA,TOPIC,6*sizeof(float)> {
    public:
      typedef utilities::packet<comm::meta,comm::meta::ID_PID_DATA,TOPIC,6*sizeof(float)> base;

    public:
      float& reset;
      float& u_p;
      float& u_i;
      float& u_d;
      float& u;
      float& e;

    public:
      pid_data(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        reset(base::template bind< float >(0)),
        u_p(base::template bind< float >(1*sizeof(float))),
        u_i(base::template bind< float >(2*sizeof(float))),
        u_d(base::template bind< float >(3*sizeof(float))),
        u(base::template bind< float >(4*sizeof(float))),
        e(base::template bind< float >(5*sizeof(float))) {
        assert(
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      pid_data(const pid_data& rhs) :
        base(rhs),
        reset(base::template bind< float >(0)),
        u_p(base::template bind< float >(1*sizeof(float))),
        u_i(base::template bind< float >(2*sizeof(float))),
        u_d(base::template bind< float >(3*sizeof(float))),
        u(base::template bind< float >(4*sizeof(float))),
        e(base::template bind< float >(5*sizeof(float))) {
        // Do nothing.
      }
      pid_data(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        reset(base::template bind< float >(0)),
        u_p(base::template bind< float >(1*sizeof(float))),
        u_i(base::template bind< float >(2*sizeof(float))),
        u_d(base::template bind< float >(3*sizeof(float))),
        u(base::template bind< float >(4*sizeof(float))),
        e(base::template bind< float >(5*sizeof(float))) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << reset << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << u_p << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << u_i << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << u_d << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << u << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << e;
        return stream;
      }
      pid_data& operator=(const pid_data& rhs) {
        base::operator=(rhs);
        return *this;
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class pid_data_DEFAULT : public pid_data<TOPIC_DEFAULT> {
      public:
        pid_data_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        pid_data_DEFAULT(const pid_data_DEFAULT& rhs);
        pid_data_DEFAULT(uint8* ptr, uint32 offset);
        ~pid_data_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
