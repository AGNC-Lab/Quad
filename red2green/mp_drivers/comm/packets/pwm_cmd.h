////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/pwm_cmd.h                                                                     //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PACKETS_PWM_CMD_H
#define PACKETS_PWM_CMD_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/packet.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace packets {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class pwm_cmd : public utilities::packet<comm::meta,comm::meta::ID_PWM_CMD,TOPIC,6*sizeof(float)> {
    public:
      typedef utilities::packet<comm::meta,comm::meta::ID_PWM_CMD,TOPIC,6*sizeof(float)> base;

    public:
      float* const p_ch; Eigen::Map< Eigen::Matrix<float,6,1> > ch;

    public:
      pwm_cmd(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        p_ch(&base::template bind< float >(0)), ch(p_ch) {
        assert(
          (TOPIC == comm::meta::DEVICE) or
          (TOPIC == comm::meta::SIM) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      pwm_cmd(const pwm_cmd& rhs) :
        base(rhs),
        p_ch(&base::template bind< float >(0)), ch(p_ch) {
        // Do nothing.
      }
      pwm_cmd(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        p_ch(&base::template bind< float >(0)), ch(p_ch) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[3] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[4] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[5];
        return stream;
      }
      pwm_cmd& operator=(const pwm_cmd& rhs) {
        base::operator=(rhs);
        return *this;
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class pwm_cmd_DEVICE : public pwm_cmd<TOPIC_DEVICE> {
      public:
        pwm_cmd_DEVICE(typename base::node_id_t node_id=comm::meta::local_node_id());
        pwm_cmd_DEVICE(const pwm_cmd_DEVICE& rhs);
        pwm_cmd_DEVICE(uint8* ptr, uint32 offset);
        ~pwm_cmd_DEVICE();
    };
    class pwm_cmd_SIM : public pwm_cmd<TOPIC_SIM> {
      public:
        pwm_cmd_SIM(typename base::node_id_t node_id=comm::meta::local_node_id());
        pwm_cmd_SIM(const pwm_cmd_SIM& rhs);
        pwm_cmd_SIM(uint8* ptr, uint32 offset);
        ~pwm_cmd_SIM();
    };
    class pwm_cmd_DEFAULT : public pwm_cmd<TOPIC_DEFAULT> {
      public:
        pwm_cmd_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        pwm_cmd_DEFAULT(const pwm_cmd_DEFAULT& rhs);
        pwm_cmd_DEFAULT(uint8* ptr, uint32 offset);
        ~pwm_cmd_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
