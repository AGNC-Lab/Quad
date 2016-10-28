////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/pos_data.h                                                                    //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PACKETS_POS_DATA_H
#define PACKETS_POS_DATA_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/packet.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace packets {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class pos_data : public utilities::packet<comm::meta,comm::meta::ID_POS_DATA,TOPIC,3*sizeof(float)> {
    public:
      typedef utilities::packet<comm::meta,comm::meta::ID_POS_DATA,TOPIC,3*sizeof(float)> base;

    public:
      float* const p_pos_ned; Eigen::Map< Eigen::Matrix<float,3,1> > pos_ned;

    public:
      pos_data(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        p_pos_ned(&base::template bind< float >(0)), pos_ned(p_pos_ned) {
        assert(
          (TOPIC == comm::meta::MOCAP) or
          (TOPIC == comm::meta::GPS) or
          (TOPIC == comm::meta::SIM) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      pos_data(const pos_data& rhs) :
        base(rhs),
        p_pos_ned(&base::template bind< float >(0)), pos_ned(p_pos_ned) {
        // Do nothing.
      }
      pos_data(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        p_pos_ned(&base::template bind< float >(0)), pos_ned(p_pos_ned) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_pos_ned[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_pos_ned[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_pos_ned[2];
        return stream;
      }
      pos_data& operator=(const pos_data& rhs) {
        base::operator=(rhs);
        return *this;
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class pos_data_MOCAP : public pos_data<TOPIC_MOCAP> {
      public:
        pos_data_MOCAP(typename base::node_id_t node_id=comm::meta::local_node_id());
        pos_data_MOCAP(const pos_data_MOCAP& rhs);
        pos_data_MOCAP(uint8* ptr, uint32 offset);
        ~pos_data_MOCAP();
    };
    class pos_data_GPS : public pos_data<TOPIC_GPS> {
      public:
        pos_data_GPS(typename base::node_id_t node_id=comm::meta::local_node_id());
        pos_data_GPS(const pos_data_GPS& rhs);
        pos_data_GPS(uint8* ptr, uint32 offset);
        ~pos_data_GPS();
    };
    class pos_data_SIM : public pos_data<TOPIC_SIM> {
      public:
        pos_data_SIM(typename base::node_id_t node_id=comm::meta::local_node_id());
        pos_data_SIM(const pos_data_SIM& rhs);
        pos_data_SIM(uint8* ptr, uint32 offset);
        ~pos_data_SIM();
    };
    class pos_data_DEFAULT : public pos_data<TOPIC_DEFAULT> {
      public:
        pos_data_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        pos_data_DEFAULT(const pos_data_DEFAULT& rhs);
        pos_data_DEFAULT(uint8* ptr, uint32 offset);
        ~pos_data_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
