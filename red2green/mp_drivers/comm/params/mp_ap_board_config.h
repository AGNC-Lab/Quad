////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/mp_ap_board_config.h                                                           //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_MP_AP_BOARD_CONFIG_H
#define PARAMS_MP_AP_BOARD_CONFIG_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/ifile.h"
#include "utilities/ofile.h"
#include "utilities/params.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace params {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class mp_ap_board_config : public utilities::params<comm::meta,comm::meta::ID_MP_AP_BOARD_CONFIG,TOPIC,sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float)> {
    public:
      typedef utilities::params<comm::meta,comm::meta::ID_MP_AP_BOARD_CONFIG,TOPIC,sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float)> base;

    public:
      uint8& imu_mux_state;
      uint8& mag_switch_state;
      uint8& alt_switch_state;

    public:
      mp_ap_board_config(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        imu_mux_state(base::template bind< uint8 >(0)),
        mag_switch_state(base::template bind< uint8 >(sizeof(uint8)+0*sizeof(float))),
        alt_switch_state(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+0*sizeof(float))) {
        assert(
          (TOPIC == comm::meta::ONBOARD) or
          (TOPIC == comm::meta::OFFBOARD) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      mp_ap_board_config(const mp_ap_board_config& rhs) :
        base(rhs),
        imu_mux_state(base::template bind< uint8 >(0)),
        mag_switch_state(base::template bind< uint8 >(sizeof(uint8)+0*sizeof(float))),
        alt_switch_state(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+0*sizeof(float))) {
        // Do nothing.
      }
      mp_ap_board_config(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        imu_mux_state(base::template bind< uint8 >(0)),
        mag_switch_state(base::template bind< uint8 >(sizeof(uint8)+0*sizeof(float))),
        alt_switch_state(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+0*sizeof(float))) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << imu_mux_state << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << mag_switch_state << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << alt_switch_state;
        return stream;
      }
      mp_ap_board_config& operator=(const mp_ap_board_config& rhs) {
        base::operator=(rhs);
        return *this;
      }

      void load_elems(const std::string& prefix, const utilities::ifile& file) {
        file.read_uint8(prefix+"imu_mux_state",imu_mux_state);
        file.read_uint8(prefix+"mag_switch_state",mag_switch_state);
        file.read_uint8(prefix+"alt_switch_state",alt_switch_state);
      }
      void save_elems(const std::string& prefix, utilities::ofile& file) const {
        file.write_uint8(prefix+"imu_mux_state",imu_mux_state);
        file.write_uint8(prefix+"mag_switch_state",mag_switch_state);
        file.write_uint8(prefix+"alt_switch_state",alt_switch_state);
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class mp_ap_board_config_ONBOARD : public mp_ap_board_config<TOPIC_ONBOARD> {
      public:
        mp_ap_board_config_ONBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        mp_ap_board_config_ONBOARD(const mp_ap_board_config_ONBOARD& rhs);
        mp_ap_board_config_ONBOARD(uint8* ptr, uint32 offset);
        ~mp_ap_board_config_ONBOARD();
    };
    class mp_ap_board_config_OFFBOARD : public mp_ap_board_config<TOPIC_OFFBOARD> {
      public:
        mp_ap_board_config_OFFBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        mp_ap_board_config_OFFBOARD(const mp_ap_board_config_OFFBOARD& rhs);
        mp_ap_board_config_OFFBOARD(uint8* ptr, uint32 offset);
        ~mp_ap_board_config_OFFBOARD();
    };
    class mp_ap_board_config_DEFAULT : public mp_ap_board_config<TOPIC_DEFAULT> {
      public:
        mp_ap_board_config_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        mp_ap_board_config_DEFAULT(const mp_ap_board_config_DEFAULT& rhs);
        mp_ap_board_config_DEFAULT(uint8* ptr, uint32 offset);
        ~mp_ap_board_config_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
