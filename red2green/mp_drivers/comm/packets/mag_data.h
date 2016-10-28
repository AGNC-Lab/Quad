////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/mag_data.h                                                                    //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PACKETS_MAG_DATA_H
#define PACKETS_MAG_DATA_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/packet.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace packets {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class mag_data : public utilities::packet<comm::meta,comm::meta::ID_MAG_DATA,TOPIC,3*sizeof(float)> {
    public:
      typedef utilities::packet<comm::meta,comm::meta::ID_MAG_DATA,TOPIC,3*sizeof(float)> base;

    public:
      float* const p_mag_vec; Eigen::Map< Eigen::Matrix<float,3,1> > mag_vec;

    public:
      mag_data(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        p_mag_vec(&base::template bind< float >(0)), mag_vec(p_mag_vec) {
        assert(
          (TOPIC == comm::meta::DEVICE) or
          (TOPIC == comm::meta::SIM) or
          (TOPIC == comm::meta::MOCAP) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      mag_data(const mag_data& rhs) :
        base(rhs),
        p_mag_vec(&base::template bind< float >(0)), mag_vec(p_mag_vec) {
        // Do nothing.
      }
      mag_data(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        p_mag_vec(&base::template bind< float >(0)), mag_vec(p_mag_vec) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_mag_vec[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_mag_vec[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_mag_vec[2];
        return stream;
      }
      mag_data& operator=(const mag_data& rhs) {
        base::operator=(rhs);
        return *this;
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class mag_data_DEVICE : public mag_data<TOPIC_DEVICE> {
      public:
        mag_data_DEVICE(typename base::node_id_t node_id=comm::meta::local_node_id());
        mag_data_DEVICE(const mag_data_DEVICE& rhs);
        mag_data_DEVICE(uint8* ptr, uint32 offset);
        ~mag_data_DEVICE();
    };
    class mag_data_SIM : public mag_data<TOPIC_SIM> {
      public:
        mag_data_SIM(typename base::node_id_t node_id=comm::meta::local_node_id());
        mag_data_SIM(const mag_data_SIM& rhs);
        mag_data_SIM(uint8* ptr, uint32 offset);
        ~mag_data_SIM();
    };
    class mag_data_MOCAP : public mag_data<TOPIC_MOCAP> {
      public:
        mag_data_MOCAP(typename base::node_id_t node_id=comm::meta::local_node_id());
        mag_data_MOCAP(const mag_data_MOCAP& rhs);
        mag_data_MOCAP(uint8* ptr, uint32 offset);
        ~mag_data_MOCAP();
    };
    class mag_data_DEFAULT : public mag_data<TOPIC_DEFAULT> {
      public:
        mag_data_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        mag_data_DEFAULT(const mag_data_DEFAULT& rhs);
        mag_data_DEFAULT(uint8* ptr, uint32 offset);
        ~mag_data_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
