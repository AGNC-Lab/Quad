////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/qctrl_params.h                                                                 //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_QCTRL_PARAMS_H
#define PARAMS_QCTRL_PARAMS_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "params/pid_gains.h"
#include "utilities/globals.h"
#include "utilities/ifile.h"
#include "utilities/ofile.h"
#include "utilities/params.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace params {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class qctrl_params : public utilities::params<comm::meta,comm::meta::ID_QCTRL_PARAMS,TOPIC,5*sizeof(float)+5*sizeof(float)+5*sizeof(float)+5*sizeof(float)+29*sizeof(float)> {
    public:
      typedef utilities::params<comm::meta,comm::meta::ID_QCTRL_PARAMS,TOPIC,5*sizeof(float)+5*sizeof(float)+5*sizeof(float)+5*sizeof(float)+29*sizeof(float)> base;

    public:
      float& cmd_timeout;
      float& zero_thrust_pwm;
      float& max_test_delta_pw;
      float& max_delta_pw;
      float& min_coll_pw;
      float& max_coll_pw;
      float& max_att_pw;
      float& max_tilt_deg;
      float& max_load_factor;
      float& max_pwm_coll_rate;
      float& max_hdg_rate_dps;
      float& min_Ts_gs;
      float& max_Ts_gs;
      float& max_delta_Ts_gs;
      float& max_omega_xy_dps;
      float& max_omega_z_dps;
      float& att_p_gain;
      float& wn_ne;
      float& zeta_ne;
      float& wn_d;
      float& zeta_d;
      float& min_altitude;
      float& max_altitude;
      float& max_north_pos;
      float& max_south_pos;
      float& max_east_pos;
      float& max_west_pos;
      float& max_hvel;
      float& max_vvel;
      params::pid_gains<TOPIC> Ts_ne_pid;
      params::pid_gains<TOPIC> Ts_z_pid;
      params::pid_gains<TOPIC> omega_xy_pid;
      params::pid_gains<TOPIC> omega_z_pid;
      float max_tilt;
      float max_hdg_rate;
      float min_Ts;
      float max_Ts;
      float max_delta_Ts;
      float max_omega_xy;
      float max_omega_z;

    public:
      qctrl_params(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        cmd_timeout(base::template bind< float >(0)),
        zero_thrust_pwm(base::template bind< float >(1*sizeof(float))),
        max_test_delta_pw(base::template bind< float >(2*sizeof(float))),
        max_delta_pw(base::template bind< float >(3*sizeof(float))),
        min_coll_pw(base::template bind< float >(4*sizeof(float))),
        max_coll_pw(base::template bind< float >(5*sizeof(float))),
        max_att_pw(base::template bind< float >(6*sizeof(float))),
        max_tilt_deg(base::template bind< float >(7*sizeof(float))),
        max_load_factor(base::template bind< float >(8*sizeof(float))),
        max_pwm_coll_rate(base::template bind< float >(9*sizeof(float))),
        max_hdg_rate_dps(base::template bind< float >(10*sizeof(float))),
        min_Ts_gs(base::template bind< float >(11*sizeof(float))),
        max_Ts_gs(base::template bind< float >(12*sizeof(float))),
        max_delta_Ts_gs(base::template bind< float >(13*sizeof(float))),
        max_omega_xy_dps(base::template bind< float >(14*sizeof(float))),
        max_omega_z_dps(base::template bind< float >(15*sizeof(float))),
        att_p_gain(base::template bind< float >(16*sizeof(float))),
        wn_ne(base::template bind< float >(17*sizeof(float))),
        zeta_ne(base::template bind< float >(18*sizeof(float))),
        wn_d(base::template bind< float >(19*sizeof(float))),
        zeta_d(base::template bind< float >(20*sizeof(float))),
        min_altitude(base::template bind< float >(21*sizeof(float))),
        max_altitude(base::template bind< float >(22*sizeof(float))),
        max_north_pos(base::template bind< float >(23*sizeof(float))),
        max_south_pos(base::template bind< float >(24*sizeof(float))),
        max_east_pos(base::template bind< float >(25*sizeof(float))),
        max_west_pos(base::template bind< float >(26*sizeof(float))),
        max_hvel(base::template bind< float >(27*sizeof(float))),
        max_vvel(base::template bind< float >(28*sizeof(float))),
        Ts_ne_pid(base::payload(),29*sizeof(float)),
        Ts_z_pid(base::payload(),5*sizeof(float)+29*sizeof(float)),
        omega_xy_pid(base::payload(),5*sizeof(float)+5*sizeof(float)+29*sizeof(float)),
        omega_z_pid(base::payload(),5*sizeof(float)+5*sizeof(float)+5*sizeof(float)+29*sizeof(float)) {
        assert(
          (TOPIC == comm::meta::ONBOARD) or
          (TOPIC == comm::meta::OFFBOARD) or
          (TOPIC == comm::meta::DEFAULT)
        );
        max_tilt = 0.0f;
        max_hdg_rate = 0.0f;
        min_Ts = 0.0f;
        max_Ts = 0.0f;
        max_delta_Ts = 0.0f;
        max_omega_xy = 0.0f;
        max_omega_z = 0.0f;
      }
      qctrl_params(const qctrl_params& rhs) :
        base(rhs),
        cmd_timeout(base::template bind< float >(0)),
        zero_thrust_pwm(base::template bind< float >(1*sizeof(float))),
        max_test_delta_pw(base::template bind< float >(2*sizeof(float))),
        max_delta_pw(base::template bind< float >(3*sizeof(float))),
        min_coll_pw(base::template bind< float >(4*sizeof(float))),
        max_coll_pw(base::template bind< float >(5*sizeof(float))),
        max_att_pw(base::template bind< float >(6*sizeof(float))),
        max_tilt_deg(base::template bind< float >(7*sizeof(float))),
        max_load_factor(base::template bind< float >(8*sizeof(float))),
        max_pwm_coll_rate(base::template bind< float >(9*sizeof(float))),
        max_hdg_rate_dps(base::template bind< float >(10*sizeof(float))),
        min_Ts_gs(base::template bind< float >(11*sizeof(float))),
        max_Ts_gs(base::template bind< float >(12*sizeof(float))),
        max_delta_Ts_gs(base::template bind< float >(13*sizeof(float))),
        max_omega_xy_dps(base::template bind< float >(14*sizeof(float))),
        max_omega_z_dps(base::template bind< float >(15*sizeof(float))),
        att_p_gain(base::template bind< float >(16*sizeof(float))),
        wn_ne(base::template bind< float >(17*sizeof(float))),
        zeta_ne(base::template bind< float >(18*sizeof(float))),
        wn_d(base::template bind< float >(19*sizeof(float))),
        zeta_d(base::template bind< float >(20*sizeof(float))),
        min_altitude(base::template bind< float >(21*sizeof(float))),
        max_altitude(base::template bind< float >(22*sizeof(float))),
        max_north_pos(base::template bind< float >(23*sizeof(float))),
        max_south_pos(base::template bind< float >(24*sizeof(float))),
        max_east_pos(base::template bind< float >(25*sizeof(float))),
        max_west_pos(base::template bind< float >(26*sizeof(float))),
        max_hvel(base::template bind< float >(27*sizeof(float))),
        max_vvel(base::template bind< float >(28*sizeof(float))),
        Ts_ne_pid(base::payload(),29*sizeof(float)),
        Ts_z_pid(base::payload(),5*sizeof(float)+29*sizeof(float)),
        omega_xy_pid(base::payload(),5*sizeof(float)+5*sizeof(float)+29*sizeof(float)),
        omega_z_pid(base::payload(),5*sizeof(float)+5*sizeof(float)+5*sizeof(float)+29*sizeof(float)) {
        max_tilt = 0.0f;
        max_hdg_rate = 0.0f;
        min_Ts = 0.0f;
        max_Ts = 0.0f;
        max_delta_Ts = 0.0f;
        max_omega_xy = 0.0f;
        max_omega_z = 0.0f;
      }
      qctrl_params(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        cmd_timeout(base::template bind< float >(0)),
        zero_thrust_pwm(base::template bind< float >(1*sizeof(float))),
        max_test_delta_pw(base::template bind< float >(2*sizeof(float))),
        max_delta_pw(base::template bind< float >(3*sizeof(float))),
        min_coll_pw(base::template bind< float >(4*sizeof(float))),
        max_coll_pw(base::template bind< float >(5*sizeof(float))),
        max_att_pw(base::template bind< float >(6*sizeof(float))),
        max_tilt_deg(base::template bind< float >(7*sizeof(float))),
        max_load_factor(base::template bind< float >(8*sizeof(float))),
        max_pwm_coll_rate(base::template bind< float >(9*sizeof(float))),
        max_hdg_rate_dps(base::template bind< float >(10*sizeof(float))),
        min_Ts_gs(base::template bind< float >(11*sizeof(float))),
        max_Ts_gs(base::template bind< float >(12*sizeof(float))),
        max_delta_Ts_gs(base::template bind< float >(13*sizeof(float))),
        max_omega_xy_dps(base::template bind< float >(14*sizeof(float))),
        max_omega_z_dps(base::template bind< float >(15*sizeof(float))),
        att_p_gain(base::template bind< float >(16*sizeof(float))),
        wn_ne(base::template bind< float >(17*sizeof(float))),
        zeta_ne(base::template bind< float >(18*sizeof(float))),
        wn_d(base::template bind< float >(19*sizeof(float))),
        zeta_d(base::template bind< float >(20*sizeof(float))),
        min_altitude(base::template bind< float >(21*sizeof(float))),
        max_altitude(base::template bind< float >(22*sizeof(float))),
        max_north_pos(base::template bind< float >(23*sizeof(float))),
        max_south_pos(base::template bind< float >(24*sizeof(float))),
        max_east_pos(base::template bind< float >(25*sizeof(float))),
        max_west_pos(base::template bind< float >(26*sizeof(float))),
        max_hvel(base::template bind< float >(27*sizeof(float))),
        max_vvel(base::template bind< float >(28*sizeof(float))),
        Ts_ne_pid(base::payload(),29*sizeof(float)),
        Ts_z_pid(base::payload(),5*sizeof(float)+29*sizeof(float)),
        omega_xy_pid(base::payload(),5*sizeof(float)+5*sizeof(float)+29*sizeof(float)),
        omega_z_pid(base::payload(),5*sizeof(float)+5*sizeof(float)+5*sizeof(float)+29*sizeof(float)) {
        max_tilt = 0.0f;
        max_hdg_rate = 0.0f;
        min_Ts = 0.0f;
        max_Ts = 0.0f;
        max_delta_Ts = 0.0f;
        max_omega_xy = 0.0f;
        max_omega_z = 0.0f;
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << cmd_timeout << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << zero_thrust_pwm << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_test_delta_pw << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_delta_pw << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << min_coll_pw << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_coll_pw << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_att_pw << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_tilt_deg << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_load_factor << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_pwm_coll_rate << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_hdg_rate_dps << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << min_Ts_gs << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_Ts_gs << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_delta_Ts_gs << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_omega_xy_dps << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_omega_z_dps << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << att_p_gain << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << wn_ne << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << zeta_ne << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << wn_d << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << zeta_d << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << min_altitude << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_altitude << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_north_pos << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_south_pos << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_east_pos << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_west_pos << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_hvel << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_vvel << utilities::serializable::DELIMITER;
        Ts_ne_pid.payload_to_stream(stream) << utilities::serializable::DELIMITER;
        Ts_z_pid.payload_to_stream(stream) << utilities::serializable::DELIMITER;
        omega_xy_pid.payload_to_stream(stream) << utilities::serializable::DELIMITER;
        omega_z_pid.payload_to_stream(stream);
        stream << " | ";
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_tilt << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_hdg_rate << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << min_Ts << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_Ts << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_delta_Ts << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_omega_xy << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << max_omega_z;
        return stream;
      }
      qctrl_params& operator=(const qctrl_params& rhs) {
        base::operator=(rhs);
        return *this;
      }

      void load_elems(const std::string& prefix, const utilities::ifile& file) {
        file.read_float(prefix+"cmd_timeout",cmd_timeout);
        file.read_float(prefix+"zero_thrust_pwm",zero_thrust_pwm);
        file.read_float(prefix+"max_test_delta_pw",max_test_delta_pw);
        file.read_float(prefix+"max_delta_pw",max_delta_pw);
        file.read_float(prefix+"min_coll_pw",min_coll_pw);
        file.read_float(prefix+"max_coll_pw",max_coll_pw);
        file.read_float(prefix+"max_att_pw",max_att_pw);
        file.read_float(prefix+"max_tilt_deg",max_tilt_deg);
        file.read_float(prefix+"max_load_factor",max_load_factor);
        file.read_float(prefix+"max_pwm_coll_rate",max_pwm_coll_rate);
        file.read_float(prefix+"max_hdg_rate_dps",max_hdg_rate_dps);
        file.read_float(prefix+"min_Ts_gs",min_Ts_gs);
        file.read_float(prefix+"max_Ts_gs",max_Ts_gs);
        file.read_float(prefix+"max_delta_Ts_gs",max_delta_Ts_gs);
        file.read_float(prefix+"max_omega_xy_dps",max_omega_xy_dps);
        file.read_float(prefix+"max_omega_z_dps",max_omega_z_dps);
        file.read_float(prefix+"att_p_gain",att_p_gain);
        file.read_float(prefix+"wn_ne",wn_ne);
        file.read_float(prefix+"zeta_ne",zeta_ne);
        file.read_float(prefix+"wn_d",wn_d);
        file.read_float(prefix+"zeta_d",zeta_d);
        file.read_float(prefix+"min_altitude",min_altitude);
        file.read_float(prefix+"max_altitude",max_altitude);
        file.read_float(prefix+"max_north_pos",max_north_pos);
        file.read_float(prefix+"max_south_pos",max_south_pos);
        file.read_float(prefix+"max_east_pos",max_east_pos);
        file.read_float(prefix+"max_west_pos",max_west_pos);
        file.read_float(prefix+"max_hvel",max_hvel);
        file.read_float(prefix+"max_vvel",max_vvel);
        Ts_ne_pid.load_elems(prefix+"Ts_ne_pid.",file);
        Ts_z_pid.load_elems(prefix+"Ts_z_pid.",file);
        omega_xy_pid.load_elems(prefix+"omega_xy_pid.",file);
        omega_z_pid.load_elems(prefix+"omega_z_pid.",file);
      }
      void save_elems(const std::string& prefix, utilities::ofile& file) const {
        file.write_float(prefix+"cmd_timeout",cmd_timeout);
        file.write_float(prefix+"zero_thrust_pwm",zero_thrust_pwm);
        file.write_float(prefix+"max_test_delta_pw",max_test_delta_pw);
        file.write_float(prefix+"max_delta_pw",max_delta_pw);
        file.write_float(prefix+"min_coll_pw",min_coll_pw);
        file.write_float(prefix+"max_coll_pw",max_coll_pw);
        file.write_float(prefix+"max_att_pw",max_att_pw);
        file.write_float(prefix+"max_tilt_deg",max_tilt_deg);
        file.write_float(prefix+"max_load_factor",max_load_factor);
        file.write_float(prefix+"max_pwm_coll_rate",max_pwm_coll_rate);
        file.write_float(prefix+"max_hdg_rate_dps",max_hdg_rate_dps);
        file.write_float(prefix+"min_Ts_gs",min_Ts_gs);
        file.write_float(prefix+"max_Ts_gs",max_Ts_gs);
        file.write_float(prefix+"max_delta_Ts_gs",max_delta_Ts_gs);
        file.write_float(prefix+"max_omega_xy_dps",max_omega_xy_dps);
        file.write_float(prefix+"max_omega_z_dps",max_omega_z_dps);
        file.write_float(prefix+"att_p_gain",att_p_gain);
        file.write_float(prefix+"wn_ne",wn_ne);
        file.write_float(prefix+"zeta_ne",zeta_ne);
        file.write_float(prefix+"wn_d",wn_d);
        file.write_float(prefix+"zeta_d",zeta_d);
        file.write_float(prefix+"min_altitude",min_altitude);
        file.write_float(prefix+"max_altitude",max_altitude);
        file.write_float(prefix+"max_north_pos",max_north_pos);
        file.write_float(prefix+"max_south_pos",max_south_pos);
        file.write_float(prefix+"max_east_pos",max_east_pos);
        file.write_float(prefix+"max_west_pos",max_west_pos);
        file.write_float(prefix+"max_hvel",max_hvel);
        file.write_float(prefix+"max_vvel",max_vvel);
        Ts_ne_pid.save_elems(prefix+"Ts_ne_pid.",file);
        Ts_z_pid.save_elems(prefix+"Ts_z_pid.",file);
        omega_xy_pid.save_elems(prefix+"omega_xy_pid.",file);
        omega_z_pid.save_elems(prefix+"omega_z_pid.",file);
      }
      void fill() {
        max_tilt = max_tilt_deg*constants::deg2rad;
        max_hdg_rate = max_hdg_rate_dps*constants::deg2rad;
        min_Ts = min_Ts_gs*constants::g0;
        max_Ts = max_Ts_gs*constants::g0;
        max_delta_Ts = max_delta_Ts_gs*constants::g0;
        max_omega_xy = max_omega_xy_dps*constants::deg2rad;
        max_omega_z = max_omega_z_dps*constants::deg2rad;
        Ts_ne_pid.fill();
        Ts_z_pid.fill();
        omega_xy_pid.fill();
        omega_z_pid.fill();
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class qctrl_params_ONBOARD : public qctrl_params<TOPIC_ONBOARD> {
      public:
        qctrl_params_ONBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        qctrl_params_ONBOARD(const qctrl_params_ONBOARD& rhs);
        qctrl_params_ONBOARD(uint8* ptr, uint32 offset);
        ~qctrl_params_ONBOARD();
    };
    class qctrl_params_OFFBOARD : public qctrl_params<TOPIC_OFFBOARD> {
      public:
        qctrl_params_OFFBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        qctrl_params_OFFBOARD(const qctrl_params_OFFBOARD& rhs);
        qctrl_params_OFFBOARD(uint8* ptr, uint32 offset);
        ~qctrl_params_OFFBOARD();
    };
    class qctrl_params_DEFAULT : public qctrl_params<TOPIC_DEFAULT> {
      public:
        qctrl_params_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        qctrl_params_DEFAULT(const qctrl_params_DEFAULT& rhs);
        qctrl_params_DEFAULT(uint8* ptr, uint32 offset);
        ~qctrl_params_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
