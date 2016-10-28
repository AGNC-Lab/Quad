////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/qsim_params.h                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_QSIM_PARAMS_H
#define PARAMS_QSIM_PARAMS_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/ifile.h"
#include "utilities/ofile.h"
#include "utilities/params.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace params {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class qsim_params : public utilities::params<comm::meta,comm::meta::ID_QSIM_PARAMS,TOPIC,31*sizeof(float)> {
    public:
      typedef utilities::params<comm::meta,comm::meta::ID_QSIM_PARAMS,TOPIC,31*sizeof(float)> base;

    public:
      float* const p_r_i_ned; Eigen::Map< Eigen::Matrix<float,3,1> > r_i_ned;
      float* const p_v_i_ned; Eigen::Map< Eigen::Matrix<float,3,1> > v_i_ned;
      float& roll_i_deg;
      float& pitch_i_deg;
      float& yaw_i_deg;
      float* const p_omega_i_dps; Eigen::Map< Eigen::Matrix<float,3,1> > omega_i_dps;
      float& m;
      float& L;
      float& cT;
      float& cM2cT;
      float& S;
      float& cD_force;
      float& cD_moment;
      float& rho;
      float& tau_up;
      float& tau_down;
      float* const p_J; Eigen::Map< Eigen::Matrix<float,3,3> > J;
      float roll_i;
      float pitch_i;
      float yaw_i;
      float p_omega_i[3]; Eigen::Map< Eigen::Matrix<float,3,1> > omega_i;
      float m_inv;
      float cT2cM;
      float tau_up_inv;
      float tau_down_inv;

    public:
      qsim_params(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        p_r_i_ned(&base::template bind< float >(0)), r_i_ned(p_r_i_ned),
        p_v_i_ned(&base::template bind< float >(3*sizeof(float))), v_i_ned(p_v_i_ned),
        roll_i_deg(base::template bind< float >(6*sizeof(float))),
        pitch_i_deg(base::template bind< float >(7*sizeof(float))),
        yaw_i_deg(base::template bind< float >(8*sizeof(float))),
        p_omega_i_dps(&base::template bind< float >(9*sizeof(float))), omega_i_dps(p_omega_i_dps),
        m(base::template bind< float >(12*sizeof(float))),
        L(base::template bind< float >(13*sizeof(float))),
        cT(base::template bind< float >(14*sizeof(float))),
        cM2cT(base::template bind< float >(15*sizeof(float))),
        S(base::template bind< float >(16*sizeof(float))),
        cD_force(base::template bind< float >(17*sizeof(float))),
        cD_moment(base::template bind< float >(18*sizeof(float))),
        rho(base::template bind< float >(19*sizeof(float))),
        tau_up(base::template bind< float >(20*sizeof(float))),
        tau_down(base::template bind< float >(21*sizeof(float))),
        p_J(&base::template bind< float >(22*sizeof(float))), J(p_J),
        omega_i(p_omega_i) {
        assert(
          (TOPIC == comm::meta::ONBOARD) or
          (TOPIC == comm::meta::OFFBOARD) or
          (TOPIC == comm::meta::DEFAULT)
        );
        roll_i = 0.0f;
        pitch_i = 0.0f;
        yaw_i = 0.0f;
        p_omega_i[0] = 0.0f;
        p_omega_i[1] = 0.0f;
        p_omega_i[2] = 0.0f;
        m_inv = 0.0f;
        cT2cM = 0.0f;
        tau_up_inv = 0.0f;
        tau_down_inv = 0.0f;
      }
      qsim_params(const qsim_params& rhs) :
        base(rhs),
        p_r_i_ned(&base::template bind< float >(0)), r_i_ned(p_r_i_ned),
        p_v_i_ned(&base::template bind< float >(3*sizeof(float))), v_i_ned(p_v_i_ned),
        roll_i_deg(base::template bind< float >(6*sizeof(float))),
        pitch_i_deg(base::template bind< float >(7*sizeof(float))),
        yaw_i_deg(base::template bind< float >(8*sizeof(float))),
        p_omega_i_dps(&base::template bind< float >(9*sizeof(float))), omega_i_dps(p_omega_i_dps),
        m(base::template bind< float >(12*sizeof(float))),
        L(base::template bind< float >(13*sizeof(float))),
        cT(base::template bind< float >(14*sizeof(float))),
        cM2cT(base::template bind< float >(15*sizeof(float))),
        S(base::template bind< float >(16*sizeof(float))),
        cD_force(base::template bind< float >(17*sizeof(float))),
        cD_moment(base::template bind< float >(18*sizeof(float))),
        rho(base::template bind< float >(19*sizeof(float))),
        tau_up(base::template bind< float >(20*sizeof(float))),
        tau_down(base::template bind< float >(21*sizeof(float))),
        p_J(&base::template bind< float >(22*sizeof(float))), J(p_J),
        omega_i(p_omega_i) {
        roll_i = 0.0f;
        pitch_i = 0.0f;
        yaw_i = 0.0f;
        p_omega_i[0] = 0.0f;
        p_omega_i[1] = 0.0f;
        p_omega_i[2] = 0.0f;
        m_inv = 0.0f;
        cT2cM = 0.0f;
        tau_up_inv = 0.0f;
        tau_down_inv = 0.0f;
      }
      qsim_params(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        p_r_i_ned(&base::template bind< float >(0)), r_i_ned(p_r_i_ned),
        p_v_i_ned(&base::template bind< float >(3*sizeof(float))), v_i_ned(p_v_i_ned),
        roll_i_deg(base::template bind< float >(6*sizeof(float))),
        pitch_i_deg(base::template bind< float >(7*sizeof(float))),
        yaw_i_deg(base::template bind< float >(8*sizeof(float))),
        p_omega_i_dps(&base::template bind< float >(9*sizeof(float))), omega_i_dps(p_omega_i_dps),
        m(base::template bind< float >(12*sizeof(float))),
        L(base::template bind< float >(13*sizeof(float))),
        cT(base::template bind< float >(14*sizeof(float))),
        cM2cT(base::template bind< float >(15*sizeof(float))),
        S(base::template bind< float >(16*sizeof(float))),
        cD_force(base::template bind< float >(17*sizeof(float))),
        cD_moment(base::template bind< float >(18*sizeof(float))),
        rho(base::template bind< float >(19*sizeof(float))),
        tau_up(base::template bind< float >(20*sizeof(float))),
        tau_down(base::template bind< float >(21*sizeof(float))),
        p_J(&base::template bind< float >(22*sizeof(float))), J(p_J),
        omega_i(p_omega_i) {
        roll_i = 0.0f;
        pitch_i = 0.0f;
        yaw_i = 0.0f;
        p_omega_i[0] = 0.0f;
        p_omega_i[1] = 0.0f;
        p_omega_i[2] = 0.0f;
        m_inv = 0.0f;
        cT2cM = 0.0f;
        tau_up_inv = 0.0f;
        tau_down_inv = 0.0f;
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_r_i_ned[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_r_i_ned[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_r_i_ned[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_v_i_ned[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_v_i_ned[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_v_i_ned[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << roll_i_deg << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << pitch_i_deg << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << yaw_i_deg << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_omega_i_dps[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_omega_i_dps[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_omega_i_dps[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << m << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << L << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << cT << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << cM2cT << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << S << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << cD_force << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << cD_moment << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << rho << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << tau_up << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << tau_down << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_J[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_J[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_J[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_J[3] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_J[4] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_J[5] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_J[6] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_J[7] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_J[8];
        stream << " | ";
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << roll_i << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << pitch_i << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << yaw_i << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_omega_i[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_omega_i[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_omega_i[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << m_inv << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << cT2cM << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << tau_up_inv << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << tau_down_inv;
        return stream;
      }
      qsim_params& operator=(const qsim_params& rhs) {
        base::operator=(rhs);
        return *this;
      }

      void load_elems(const std::string& prefix, const utilities::ifile& file) {
        file.read_float(prefix+"r_i_ned[0]",p_r_i_ned[0]);
        file.read_float(prefix+"r_i_ned[1]",p_r_i_ned[1]);
        file.read_float(prefix+"r_i_ned[2]",p_r_i_ned[2]);
        file.read_float(prefix+"v_i_ned[0]",p_v_i_ned[0]);
        file.read_float(prefix+"v_i_ned[1]",p_v_i_ned[1]);
        file.read_float(prefix+"v_i_ned[2]",p_v_i_ned[2]);
        file.read_float(prefix+"roll_i_deg",roll_i_deg);
        file.read_float(prefix+"pitch_i_deg",pitch_i_deg);
        file.read_float(prefix+"yaw_i_deg",yaw_i_deg);
        file.read_float(prefix+"omega_i_dps[0]",p_omega_i_dps[0]);
        file.read_float(prefix+"omega_i_dps[1]",p_omega_i_dps[1]);
        file.read_float(prefix+"omega_i_dps[2]",p_omega_i_dps[2]);
        file.read_float(prefix+"m",m);
        file.read_float(prefix+"L",L);
        file.read_float(prefix+"cT",cT);
        file.read_float(prefix+"cM2cT",cM2cT);
        file.read_float(prefix+"S",S);
        file.read_float(prefix+"cD_force",cD_force);
        file.read_float(prefix+"cD_moment",cD_moment);
        file.read_float(prefix+"rho",rho);
        file.read_float(prefix+"tau_up",tau_up);
        file.read_float(prefix+"tau_down",tau_down);
        file.read_float(prefix+"J[0]",p_J[0]);
        file.read_float(prefix+"J[1]",p_J[1]);
        file.read_float(prefix+"J[2]",p_J[2]);
        file.read_float(prefix+"J[3]",p_J[3]);
        file.read_float(prefix+"J[4]",p_J[4]);
        file.read_float(prefix+"J[5]",p_J[5]);
        file.read_float(prefix+"J[6]",p_J[6]);
        file.read_float(prefix+"J[7]",p_J[7]);
        file.read_float(prefix+"J[8]",p_J[8]);
      }
      void save_elems(const std::string& prefix, utilities::ofile& file) const {
        file.write_float(prefix+"r_i_ned[0]",p_r_i_ned[0]);
        file.write_float(prefix+"r_i_ned[1]",p_r_i_ned[1]);
        file.write_float(prefix+"r_i_ned[2]",p_r_i_ned[2]);
        file.write_float(prefix+"v_i_ned[0]",p_v_i_ned[0]);
        file.write_float(prefix+"v_i_ned[1]",p_v_i_ned[1]);
        file.write_float(prefix+"v_i_ned[2]",p_v_i_ned[2]);
        file.write_float(prefix+"roll_i_deg",roll_i_deg);
        file.write_float(prefix+"pitch_i_deg",pitch_i_deg);
        file.write_float(prefix+"yaw_i_deg",yaw_i_deg);
        file.write_float(prefix+"omega_i_dps[0]",p_omega_i_dps[0]);
        file.write_float(prefix+"omega_i_dps[1]",p_omega_i_dps[1]);
        file.write_float(prefix+"omega_i_dps[2]",p_omega_i_dps[2]);
        file.write_float(prefix+"m",m);
        file.write_float(prefix+"L",L);
        file.write_float(prefix+"cT",cT);
        file.write_float(prefix+"cM2cT",cM2cT);
        file.write_float(prefix+"S",S);
        file.write_float(prefix+"cD_force",cD_force);
        file.write_float(prefix+"cD_moment",cD_moment);
        file.write_float(prefix+"rho",rho);
        file.write_float(prefix+"tau_up",tau_up);
        file.write_float(prefix+"tau_down",tau_down);
        file.write_float(prefix+"J[0]",p_J[0]);
        file.write_float(prefix+"J[1]",p_J[1]);
        file.write_float(prefix+"J[2]",p_J[2]);
        file.write_float(prefix+"J[3]",p_J[3]);
        file.write_float(prefix+"J[4]",p_J[4]);
        file.write_float(prefix+"J[5]",p_J[5]);
        file.write_float(prefix+"J[6]",p_J[6]);
        file.write_float(prefix+"J[7]",p_J[7]);
        file.write_float(prefix+"J[8]",p_J[8]);
      }
      void fill() {
        roll_i = roll_i_deg*constants::deg2rad;
        pitch_i = pitch_i_deg*constants::deg2rad;
        yaw_i = yaw_i_deg*constants::deg2rad;
        p_omega_i[0] = p_omega_i_dps[0]*constants::deg2rad;
        p_omega_i[1] = p_omega_i_dps[1]*constants::deg2rad;
        p_omega_i[2] = p_omega_i_dps[2]*constants::deg2rad;
        m_inv = pow(m,-1.0);
        cT2cM = pow(cM2cT,-1.0);
        tau_up_inv = pow(tau_up,-1.0);
        tau_down_inv = pow(tau_down,-1.0);
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class qsim_params_ONBOARD : public qsim_params<TOPIC_ONBOARD> {
      public:
        qsim_params_ONBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        qsim_params_ONBOARD(const qsim_params_ONBOARD& rhs);
        qsim_params_ONBOARD(uint8* ptr, uint32 offset);
        ~qsim_params_ONBOARD();
    };
    class qsim_params_OFFBOARD : public qsim_params<TOPIC_OFFBOARD> {
      public:
        qsim_params_OFFBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        qsim_params_OFFBOARD(const qsim_params_OFFBOARD& rhs);
        qsim_params_OFFBOARD(uint8* ptr, uint32 offset);
        ~qsim_params_OFFBOARD();
    };
    class qsim_params_DEFAULT : public qsim_params<TOPIC_DEFAULT> {
      public:
        qsim_params_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        qsim_params_DEFAULT(const qsim_params_DEFAULT& rhs);
        qsim_params_DEFAULT(uint8* ptr, uint32 offset);
        ~qsim_params_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
