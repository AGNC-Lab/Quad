////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/quat.h                                                                        //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PACKETS_QUAT_H
#define PACKETS_QUAT_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/packet.h"
#include "utilities/quat.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace packets {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class quat : public utilities::packet<comm::meta,comm::meta::ID_QUAT,TOPIC,sizeof(utilities::quat<float>)+0*sizeof(float)> {
    public:
      typedef utilities::packet<comm::meta,comm::meta::ID_QUAT,TOPIC,sizeof(utilities::quat<float>)+0*sizeof(float)> base;

    public:
      utilities::quat<float>& q;

    public:
      quat(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        q(base::template bind< utilities::quat<float> >(0)) {
        q = utilities::quat<float>();
        assert(
          (TOPIC == comm::meta::AHRS) or
          (TOPIC == comm::meta::QSIM) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      quat(const quat& rhs) :
        base(rhs),
        q(base::template bind< utilities::quat<float> >(0)) {
        q = utilities::quat<float>();
      }
      quat(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        q(base::template bind< utilities::quat<float> >(0)) {
        q = utilities::quat<float>();
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << q;
        return stream;
      }
      quat& operator=(const quat& rhs) {
        base::operator=(rhs);
        return *this;
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class quat_AHRS : public quat<TOPIC_AHRS> {
      public:
        quat_AHRS(typename base::node_id_t node_id=comm::meta::local_node_id());
        quat_AHRS(const quat_AHRS& rhs);
        quat_AHRS(uint8* ptr, uint32 offset);
        ~quat_AHRS();
    };
    class quat_QSIM : public quat<TOPIC_QSIM> {
      public:
        quat_QSIM(typename base::node_id_t node_id=comm::meta::local_node_id());
        quat_QSIM(const quat_QSIM& rhs);
        quat_QSIM(uint8* ptr, uint32 offset);
        ~quat_QSIM();
    };
    class quat_DEFAULT : public quat<TOPIC_DEFAULT> {
      public:
        quat_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        quat_DEFAULT(const quat_DEFAULT& rhs);
        quat_DEFAULT(uint8* ptr, uint32 offset);
        ~quat_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
