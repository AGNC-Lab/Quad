////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/qctrl_cmd.h                                                                   //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PACKETS_QCTRL_CMD_H
#define PACKETS_QCTRL_CMD_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "states/ctrl_auth_mode.h"
#include "states/qctrl_mode.h"
#include "utilities/globals.h"
#include "utilities/packet.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace packets {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class qctrl_cmd : public utilities::packet<comm::meta,comm::meta::ID_QCTRL_CMD,TOPIC,1+1+10*sizeof(float)> {
    public:
      typedef utilities::packet<comm::meta,comm::meta::ID_QCTRL_CMD,TOPIC,1+1+10*sizeof(float)> base;

    public:
      states::ctrl_auth_mode::state<TOPIC> auth_src;
      states::qctrl_mode::state<TOPIC> mode;
      float* const p_ch; Eigen::Map< Eigen::Matrix<float,10,1> > ch;

    public:
      qctrl_cmd(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        auth_src(base::payload(),0),
        mode(base::payload(),1+0*sizeof(float)),
        p_ch(&base::template bind< float >(1+1+0*sizeof(float))), ch(p_ch) {
        assert(
          (TOPIC == comm::meta::GCS) or
          (TOPIC == comm::meta::API) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      qctrl_cmd(const qctrl_cmd& rhs) :
        base(rhs),
        auth_src(base::payload(),0),
        mode(base::payload(),1+0*sizeof(float)),
        p_ch(&base::template bind< float >(1+1+0*sizeof(float))), ch(p_ch) {
        // Do nothing.
      }
      qctrl_cmd(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        auth_src(base::payload(),0),
        mode(base::payload(),1+0*sizeof(float)),
        p_ch(&base::template bind< float >(1+1+0*sizeof(float))), ch(p_ch) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        auth_src.payload_to_stream(stream) << utilities::serializable::DELIMITER;
        mode.payload_to_stream(stream) << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[3] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[4] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[5] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[6] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[7] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[8] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_ch[9];
        return stream;
      }
      qctrl_cmd& operator=(const qctrl_cmd& rhs) {
        base::operator=(rhs);
        return *this;
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class qctrl_cmd_GCS : public qctrl_cmd<TOPIC_GCS> {
      public:
        qctrl_cmd_GCS(typename base::node_id_t node_id=comm::meta::local_node_id());
        qctrl_cmd_GCS(const qctrl_cmd_GCS& rhs);
        qctrl_cmd_GCS(uint8* ptr, uint32 offset);
        ~qctrl_cmd_GCS();
    };
    class qctrl_cmd_API : public qctrl_cmd<TOPIC_API> {
      public:
        qctrl_cmd_API(typename base::node_id_t node_id=comm::meta::local_node_id());
        qctrl_cmd_API(const qctrl_cmd_API& rhs);
        qctrl_cmd_API(uint8* ptr, uint32 offset);
        ~qctrl_cmd_API();
    };
    class qctrl_cmd_DEFAULT : public qctrl_cmd<TOPIC_DEFAULT> {
      public:
        qctrl_cmd_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        qctrl_cmd_DEFAULT(const qctrl_cmd_DEFAULT& rhs);
        qctrl_cmd_DEFAULT(uint8* ptr, uint32 offset);
        ~qctrl_cmd_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
