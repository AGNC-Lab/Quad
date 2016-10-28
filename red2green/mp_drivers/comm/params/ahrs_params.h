////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/ahrs_params.h                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_AHRS_PARAMS_H
#define PARAMS_AHRS_PARAMS_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/ifile.h"
#include "utilities/ofile.h"
#include "utilities/params.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace params {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class ahrs_params : public utilities::params<comm::meta,comm::meta::ID_AHRS_PARAMS,TOPIC,sizeof(uint32)+sizeof(uint32)+11*sizeof(float)> {
    public:
      typedef utilities::params<comm::meta,comm::meta::ID_AHRS_PARAMS,TOPIC,sizeof(uint32)+sizeof(uint32)+11*sizeof(float)> base;

    public:
      float& std_P_qq_0;
      float& std_P_bb_0_dps;
      float& std_gyro_dps;
      float& std_b_gyro_dps2;
      float& std_pseudo_q_norm;
      float& std_grav_gs;
      float& std_hdg_deg;
      uint32& n_imu_samples;
      uint32& n_mag_samples;
      float& tau;
      float& grav_update_threshold_gs;
      float& max_hdg_innov_deg;
      float& mag_dec_deg;
      float var_P_qq_0;
      float var_P_bb_0;
      float var_gyro;
      float var_b_gyro;
      float var_pseudo_q_norm;
      float var_grav_gs;
      float var_hdg;
      float tau_inv;
      float max_hdg_innov;
      float mag_dec;

    public:
      ahrs_params(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        std_P_qq_0(base::template bind< float >(0)),
        std_P_bb_0_dps(base::template bind< float >(1*sizeof(float))),
        std_gyro_dps(base::template bind< float >(2*sizeof(float))),
        std_b_gyro_dps2(base::template bind< float >(3*sizeof(float))),
        std_pseudo_q_norm(base::template bind< float >(4*sizeof(float))),
        std_grav_gs(base::template bind< float >(5*sizeof(float))),
        std_hdg_deg(base::template bind< float >(6*sizeof(float))),
        n_imu_samples(base::template bind< uint32 >(7*sizeof(float))),
        n_mag_samples(base::template bind< uint32 >(sizeof(uint32)+7*sizeof(float))),
        tau(base::template bind< float >(sizeof(uint32)+sizeof(uint32)+7*sizeof(float))),
        grav_update_threshold_gs(base::template bind< float >(sizeof(uint32)+sizeof(uint32)+8*sizeof(float))),
        max_hdg_innov_deg(base::template bind< float >(sizeof(uint32)+sizeof(uint32)+9*sizeof(float))),
        mag_dec_deg(base::template bind< float >(sizeof(uint32)+sizeof(uint32)+10*sizeof(float))) {
        assert(
          (TOPIC == comm::meta::ONBOARD) or
          (TOPIC == comm::meta::OFFBOARD) or
          (TOPIC == comm::meta::DEFAULT)
        );
        var_P_qq_0 = 0.0f;
        var_P_bb_0 = 0.0f;
        var_gyro = 0.0f;
        var_b_gyro = 0.0f;
        var_pseudo_q_norm = 0.0f;
        var_grav_gs = 0.0f;
        var_hdg = 0.0f;
        tau_inv = 0.0f;
        max_hdg_innov = 0.0f;
        mag_dec = 0.0f;
      }
      ahrs_params(const ahrs_params& rhs) :
        base(rhs),
        std_P_qq_0(base::template bind< float >(0)),
        std_P_bb_0_dps(base::template bind< float >(1*sizeof(float))),
        std_gyro_dps(base::template bind< float >(2*sizeof(float))),
        std_b_gyro_dps2(base::template bind< float >(3*sizeof(float))),
        std_pseudo_q_norm(base::template bind< float >(4*sizeof(float))),
        std_grav_gs(base::template bind< float >(5*sizeof(float))),
        std_hdg_deg(base::template bind< float >(6*sizeof(float))),
        n_imu_samples(base::template bind< uint32 >(7*sizeof(float))),
        n_mag_samples(base::template bind< uint32 >(sizeof(uint32)+7*sizeof(float))),
        tau(base::template bind< float >(sizeof(uint32)+sizeof(uint32)+7*sizeof(float))),
        grav_update_threshold_gs(base::template bind< float >(sizeof(uint32)+sizeof(uint32)+8*sizeof(float))),
        max_hdg_innov_deg(base::template bind< float >(sizeof(uint32)+sizeof(uint32)+9*sizeof(float))),
        mag_dec_deg(base::template bind< float >(sizeof(uint32)+sizeof(uint32)+10*sizeof(float))) {
        var_P_qq_0 = 0.0f;
        var_P_bb_0 = 0.0f;
        var_gyro = 0.0f;
        var_b_gyro = 0.0f;
        var_pseudo_q_norm = 0.0f;
        var_grav_gs = 0.0f;
        var_hdg = 0.0f;
        tau_inv = 0.0f;
        max_hdg_innov = 0.0f;
        mag_dec = 0.0f;
      }
      ahrs_params(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        std_P_qq_0(base::template bind< float >(0)),
        std_P_bb_0_dps(base::template bind< float >(1*sizeof(float))),
        std_gyro_dps(base::template bind< float >(2*sizeof(float))),
        std_b_gyro_dps2(base::template bind< float >(3*sizeof(float))),
        std_pseudo_q_norm(base::template bind< float >(4*sizeof(float))),
        std_grav_gs(base::template bind< float >(5*sizeof(float))),
        std_hdg_deg(base::template bind< float >(6*sizeof(float))),
        n_imu_samples(base::template bind< uint32 >(7*sizeof(float))),
        n_mag_samples(base::template bind< uint32 >(sizeof(uint32)+7*sizeof(float))),
        tau(base::template bind< float >(sizeof(uint32)+sizeof(uint32)+7*sizeof(float))),
        grav_update_threshold_gs(base::template bind< float >(sizeof(uint32)+sizeof(uint32)+8*sizeof(float))),
        max_hdg_innov_deg(base::template bind< float >(sizeof(uint32)+sizeof(uint32)+9*sizeof(float))),
        mag_dec_deg(base::template bind< float >(sizeof(uint32)+sizeof(uint32)+10*sizeof(float))) {
        var_P_qq_0 = 0.0f;
        var_P_bb_0 = 0.0f;
        var_gyro = 0.0f;
        var_b_gyro = 0.0f;
        var_pseudo_q_norm = 0.0f;
        var_grav_gs = 0.0f;
        var_hdg = 0.0f;
        tau_inv = 0.0f;
        max_hdg_innov = 0.0f;
        mag_dec = 0.0f;
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << std_P_qq_0 << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << std_P_bb_0_dps << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << std_gyro_dps << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << std_b_gyro_dps2 << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << std_pseudo_q_norm << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << std_grav_gs << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << std_hdg_deg << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << n_imu_samples << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << n_mag_samples << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << tau << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << grav_update_threshold_gs << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_hdg_innov_deg << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << mag_dec_deg;
        stream << " | ";
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << var_P_qq_0 << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << var_P_bb_0 << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << var_gyro << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << var_b_gyro << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << var_pseudo_q_norm << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << var_grav_gs << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << var_hdg << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << tau_inv << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_hdg_innov << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << mag_dec;
        return stream;
      }
      ahrs_params& operator=(const ahrs_params& rhs) {
        base::operator=(rhs);
        return *this;
      }

      void load_elems(const std::string& prefix, const utilities::ifile& file) {
        file.read_float(prefix+"std_P_qq_0",std_P_qq_0);
        file.read_float(prefix+"std_P_bb_0_dps",std_P_bb_0_dps);
        file.read_float(prefix+"std_gyro_dps",std_gyro_dps);
        file.read_float(prefix+"std_b_gyro_dps2",std_b_gyro_dps2);
        file.read_float(prefix+"std_pseudo_q_norm",std_pseudo_q_norm);
        file.read_float(prefix+"std_grav_gs",std_grav_gs);
        file.read_float(prefix+"std_hdg_deg",std_hdg_deg);
        file.read_uint32(prefix+"n_imu_samples",n_imu_samples);
        file.read_uint32(prefix+"n_mag_samples",n_mag_samples);
        file.read_float(prefix+"tau",tau);
        file.read_float(prefix+"grav_update_threshold_gs",grav_update_threshold_gs);
        file.read_float(prefix+"max_hdg_innov_deg",max_hdg_innov_deg);
        file.read_float(prefix+"mag_dec_deg",mag_dec_deg);
      }
      void save_elems(const std::string& prefix, utilities::ofile& file) const {
        file.write_float(prefix+"std_P_qq_0",std_P_qq_0);
        file.write_float(prefix+"std_P_bb_0_dps",std_P_bb_0_dps);
        file.write_float(prefix+"std_gyro_dps",std_gyro_dps);
        file.write_float(prefix+"std_b_gyro_dps2",std_b_gyro_dps2);
        file.write_float(prefix+"std_pseudo_q_norm",std_pseudo_q_norm);
        file.write_float(prefix+"std_grav_gs",std_grav_gs);
        file.write_float(prefix+"std_hdg_deg",std_hdg_deg);
        file.write_uint32(prefix+"n_imu_samples",n_imu_samples);
        file.write_uint32(prefix+"n_mag_samples",n_mag_samples);
        file.write_float(prefix+"tau",tau);
        file.write_float(prefix+"grav_update_threshold_gs",grav_update_threshold_gs);
        file.write_float(prefix+"max_hdg_innov_deg",max_hdg_innov_deg);
        file.write_float(prefix+"mag_dec_deg",mag_dec_deg);
      }
      void fill() {
        var_P_qq_0 = pow(std_P_qq_0,2.0);
        var_P_bb_0 = pow(std_P_bb_0_dps*constants::deg2rad,2.0);
        var_gyro = pow(std_gyro_dps*constants::deg2rad,2.0);
        var_b_gyro = pow(std_b_gyro_dps2*constants::deg2rad,2.0);
        var_pseudo_q_norm = pow(std_pseudo_q_norm,2.0);
        var_grav_gs = pow(std_grav_gs,2.0);
        var_hdg = pow(std_hdg_deg*constants::deg2rad,2.0);
        tau_inv = pow(tau,-1.0);
        max_hdg_innov = max_hdg_innov_deg*constants::deg2rad;
        mag_dec = mag_dec_deg*constants::deg2rad;
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class ahrs_params_ONBOARD : public ahrs_params<TOPIC_ONBOARD> {
      public:
        ahrs_params_ONBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        ahrs_params_ONBOARD(const ahrs_params_ONBOARD& rhs);
        ahrs_params_ONBOARD(uint8* ptr, uint32 offset);
        ~ahrs_params_ONBOARD();
    };
    class ahrs_params_OFFBOARD : public ahrs_params<TOPIC_OFFBOARD> {
      public:
        ahrs_params_OFFBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        ahrs_params_OFFBOARD(const ahrs_params_OFFBOARD& rhs);
        ahrs_params_OFFBOARD(uint8* ptr, uint32 offset);
        ~ahrs_params_OFFBOARD();
    };
    class ahrs_params_DEFAULT : public ahrs_params<TOPIC_DEFAULT> {
      public:
        ahrs_params_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        ahrs_params_DEFAULT(const ahrs_params_DEFAULT& rhs);
        ahrs_params_DEFAULT(uint8* ptr, uint32 offset);
        ~ahrs_params_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
