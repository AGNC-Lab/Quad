////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/overo_config.h                                                                 //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_OVERO_CONFIG_H
#define PARAMS_OVERO_CONFIG_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/ifile.h"
#include "utilities/ofile.h"
#include "utilities/params.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace params {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class overo_config : public utilities::params<comm::meta,comm::meta::ID_OVERO_CONFIG,TOPIC,sizeof(uint8)+sizeof(uint8)+0*sizeof(float)> {
    public:
      typedef utilities::params<comm::meta,comm::meta::ID_OVERO_CONFIG,TOPIC,sizeof(uint8)+sizeof(uint8)+0*sizeof(float)> base;

    public:
      uint8& timeout_msec;
      uint8& pw_filt_factor;

    public:
      overo_config(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        timeout_msec(base::template bind< uint8 >(0)),
        pw_filt_factor(base::template bind< uint8 >(sizeof(uint8)+0*sizeof(float))) {
        assert(
          (TOPIC == comm::meta::ONBOARD) or
          (TOPIC == comm::meta::OFFBOARD) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      overo_config(const overo_config& rhs) :
        base(rhs),
        timeout_msec(base::template bind< uint8 >(0)),
        pw_filt_factor(base::template bind< uint8 >(sizeof(uint8)+0*sizeof(float))) {
        // Do nothing.
      }
      overo_config(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        timeout_msec(base::template bind< uint8 >(0)),
        pw_filt_factor(base::template bind< uint8 >(sizeof(uint8)+0*sizeof(float))) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << timeout_msec << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << pw_filt_factor;
        return stream;
      }
      overo_config& operator=(const overo_config& rhs) {
        base::operator=(rhs);
        return *this;
      }

      void load_elems(const std::string& prefix, const utilities::ifile& file) {
        file.read_uint8(prefix+"timeout_msec",timeout_msec);
        file.read_uint8(prefix+"pw_filt_factor",pw_filt_factor);
      }
      void save_elems(const std::string& prefix, utilities::ofile& file) const {
        file.write_uint8(prefix+"timeout_msec",timeout_msec);
        file.write_uint8(prefix+"pw_filt_factor",pw_filt_factor);
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class overo_config_ONBOARD : public overo_config<TOPIC_ONBOARD> {
      public:
        overo_config_ONBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        overo_config_ONBOARD(const overo_config_ONBOARD& rhs);
        overo_config_ONBOARD(uint8* ptr, uint32 offset);
        ~overo_config_ONBOARD();
    };
    class overo_config_OFFBOARD : public overo_config<TOPIC_OFFBOARD> {
      public:
        overo_config_OFFBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        overo_config_OFFBOARD(const overo_config_OFFBOARD& rhs);
        overo_config_OFFBOARD(uint8* ptr, uint32 offset);
        ~overo_config_OFFBOARD();
    };
    class overo_config_DEFAULT : public overo_config<TOPIC_DEFAULT> {
      public:
        overo_config_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        overo_config_DEFAULT(const overo_config_DEFAULT& rhs);
        overo_config_DEFAULT(uint8* ptr, uint32 offset);
        ~overo_config_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
