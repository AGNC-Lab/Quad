////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/qctrl_data.h                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PACKETS_QCTRL_DATA_H
#define PACKETS_QCTRL_DATA_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "packets/pid_data.h"
#include "utilities/globals.h"
#include "utilities/packet.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace packets {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class qctrl_data : public utilities::packet<comm::meta,comm::meta::ID_QCTRL_DATA,TOPIC,6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+10*sizeof(float)> {
    public:
      typedef utilities::packet<comm::meta,comm::meta::ID_QCTRL_DATA,TOPIC,6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+10*sizeof(float)> base;

    public:
      float& ts_last;
      float& dt;
      float& pwm_coll;
      float& hdg_ref;
      float* const p_pos_ref_ned; Eigen::Map< Eigen::Matrix<float,3,1> > pos_ref_ned;
      float* const p_vel_ned; Eigen::Map< Eigen::Matrix<float,3,1> > vel_ned;
      packets::pid_data<TOPIC> Ts_n_pid;
      packets::pid_data<TOPIC> Ts_e_pid;
      packets::pid_data<TOPIC> Ts_z_pid;
      packets::pid_data<TOPIC> omega_x_pid;
      packets::pid_data<TOPIC> omega_y_pid;
      packets::pid_data<TOPIC> omega_z_pid;

    public:
      qctrl_data(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        ts_last(base::template bind< float >(0)),
        dt(base::template bind< float >(1*sizeof(float))),
        pwm_coll(base::template bind< float >(2*sizeof(float))),
        hdg_ref(base::template bind< float >(3*sizeof(float))),
        p_pos_ref_ned(&base::template bind< float >(4*sizeof(float))), pos_ref_ned(p_pos_ref_ned),
        p_vel_ned(&base::template bind< float >(7*sizeof(float))), vel_ned(p_vel_ned),
        Ts_n_pid(base::payload(),10*sizeof(float)),
        Ts_e_pid(base::payload(),6*sizeof(float)+10*sizeof(float)),
        Ts_z_pid(base::payload(),6*sizeof(float)+6*sizeof(float)+10*sizeof(float)),
        omega_x_pid(base::payload(),6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+10*sizeof(float)),
        omega_y_pid(base::payload(),6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+10*sizeof(float)),
        omega_z_pid(base::payload(),6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+10*sizeof(float)) {
        assert(
          (TOPIC == comm::meta::QCTRL) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      qctrl_data(const qctrl_data& rhs) :
        base(rhs),
        ts_last(base::template bind< float >(0)),
        dt(base::template bind< float >(1*sizeof(float))),
        pwm_coll(base::template bind< float >(2*sizeof(float))),
        hdg_ref(base::template bind< float >(3*sizeof(float))),
        p_pos_ref_ned(&base::template bind< float >(4*sizeof(float))), pos_ref_ned(p_pos_ref_ned),
        p_vel_ned(&base::template bind< float >(7*sizeof(float))), vel_ned(p_vel_ned),
        Ts_n_pid(base::payload(),10*sizeof(float)),
        Ts_e_pid(base::payload(),6*sizeof(float)+10*sizeof(float)),
        Ts_z_pid(base::payload(),6*sizeof(float)+6*sizeof(float)+10*sizeof(float)),
        omega_x_pid(base::payload(),6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+10*sizeof(float)),
        omega_y_pid(base::payload(),6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+10*sizeof(float)),
        omega_z_pid(base::payload(),6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+10*sizeof(float)) {
        // Do nothing.
      }
      qctrl_data(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        ts_last(base::template bind< float >(0)),
        dt(base::template bind< float >(1*sizeof(float))),
        pwm_coll(base::template bind< float >(2*sizeof(float))),
        hdg_ref(base::template bind< float >(3*sizeof(float))),
        p_pos_ref_ned(&base::template bind< float >(4*sizeof(float))), pos_ref_ned(p_pos_ref_ned),
        p_vel_ned(&base::template bind< float >(7*sizeof(float))), vel_ned(p_vel_ned),
        Ts_n_pid(base::payload(),10*sizeof(float)),
        Ts_e_pid(base::payload(),6*sizeof(float)+10*sizeof(float)),
        Ts_z_pid(base::payload(),6*sizeof(float)+6*sizeof(float)+10*sizeof(float)),
        omega_x_pid(base::payload(),6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+10*sizeof(float)),
        omega_y_pid(base::payload(),6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+10*sizeof(float)),
        omega_z_pid(base::payload(),6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+6*sizeof(float)+10*sizeof(float)) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << ts_last << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << dt << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << pwm_coll << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << hdg_ref << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_pos_ref_ned[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_pos_ref_ned[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_pos_ref_ned[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_vel_ned[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_vel_ned[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_vel_ned[2] << utilities::serializable::DELIMITER;
        Ts_n_pid.payload_to_stream(stream) << utilities::serializable::DELIMITER;
        Ts_e_pid.payload_to_stream(stream) << utilities::serializable::DELIMITER;
        Ts_z_pid.payload_to_stream(stream) << utilities::serializable::DELIMITER;
        omega_x_pid.payload_to_stream(stream) << utilities::serializable::DELIMITER;
        omega_y_pid.payload_to_stream(stream) << utilities::serializable::DELIMITER;
        omega_z_pid.payload_to_stream(stream);
        return stream;
      }
      qctrl_data& operator=(const qctrl_data& rhs) {
        base::operator=(rhs);
        return *this;
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class qctrl_data_QCTRL : public qctrl_data<TOPIC_QCTRL> {
      public:
        qctrl_data_QCTRL(typename base::node_id_t node_id=comm::meta::local_node_id());
        qctrl_data_QCTRL(const qctrl_data_QCTRL& rhs);
        qctrl_data_QCTRL(uint8* ptr, uint32 offset);
        ~qctrl_data_QCTRL();
    };
    class qctrl_data_DEFAULT : public qctrl_data<TOPIC_DEFAULT> {
      public:
        qctrl_data_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        qctrl_data_DEFAULT(const qctrl_data_DEFAULT& rhs);
        qctrl_data_DEFAULT(uint8* ptr, uint32 offset);
        ~qctrl_data_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
