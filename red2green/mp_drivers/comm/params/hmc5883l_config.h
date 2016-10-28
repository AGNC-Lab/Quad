////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/hmc5883l_config.h                                                              //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_HMC5883L_CONFIG_H
#define PARAMS_HMC5883L_CONFIG_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/ifile.h"
#include "utilities/ofile.h"
#include "utilities/params.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace params {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class hmc5883l_config : public utilities::params<comm::meta,comm::meta::ID_HMC5883L_CONFIG,TOPIC,sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint32)+9*sizeof(float)> {
    public:
      typedef utilities::params<comm::meta,comm::meta::ID_HMC5883L_CONFIG,TOPIC,sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint32)+9*sizeof(float)> base;

    public:
      uint8& op_mode;
      uint8& avg_samp;
      uint8& rate;
      uint8& meas_mode;
      uint8& fs_mag;
      uint32& n_calib_samples;
      float* const p_C_dev2body; Eigen::Map< Eigen::Matrix<float,3,3> > C_dev2body;

    public:
      hmc5883l_config(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        op_mode(base::template bind< uint8 >(0)),
        avg_samp(base::template bind< uint8 >(sizeof(uint8)+0*sizeof(float))),
        rate(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        meas_mode(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        fs_mag(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        n_calib_samples(base::template bind< uint32 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        p_C_dev2body(&base::template bind< float >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint32)+0*sizeof(float))), C_dev2body(p_C_dev2body) {
        assert(
          (TOPIC == comm::meta::ONBOARD) or
          (TOPIC == comm::meta::OFFBOARD) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      hmc5883l_config(const hmc5883l_config& rhs) :
        base(rhs),
        op_mode(base::template bind< uint8 >(0)),
        avg_samp(base::template bind< uint8 >(sizeof(uint8)+0*sizeof(float))),
        rate(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        meas_mode(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        fs_mag(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        n_calib_samples(base::template bind< uint32 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        p_C_dev2body(&base::template bind< float >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint32)+0*sizeof(float))), C_dev2body(p_C_dev2body) {
        // Do nothing.
      }
      hmc5883l_config(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        op_mode(base::template bind< uint8 >(0)),
        avg_samp(base::template bind< uint8 >(sizeof(uint8)+0*sizeof(float))),
        rate(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        meas_mode(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        fs_mag(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        n_calib_samples(base::template bind< uint32 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        p_C_dev2body(&base::template bind< float >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint32)+0*sizeof(float))), C_dev2body(p_C_dev2body) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << op_mode << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << avg_samp << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << rate << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << meas_mode << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << fs_mag << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << n_calib_samples << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_C_dev2body[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_C_dev2body[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_C_dev2body[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_C_dev2body[3] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_C_dev2body[4] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_C_dev2body[5] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_C_dev2body[6] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_C_dev2body[7] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_C_dev2body[8];
        return stream;
      }
      hmc5883l_config& operator=(const hmc5883l_config& rhs) {
        base::operator=(rhs);
        return *this;
      }

      void load_elems(const std::string& prefix, const utilities::ifile& file) {
        file.read_uint8(prefix+"op_mode",op_mode);
        file.read_uint8(prefix+"avg_samp",avg_samp);
        file.read_uint8(prefix+"rate",rate);
        file.read_uint8(prefix+"meas_mode",meas_mode);
        file.read_uint8(prefix+"fs_mag",fs_mag);
        file.read_uint32(prefix+"n_calib_samples",n_calib_samples);
        file.read_float(prefix+"C_dev2body[0]",p_C_dev2body[0]);
        file.read_float(prefix+"C_dev2body[1]",p_C_dev2body[1]);
        file.read_float(prefix+"C_dev2body[2]",p_C_dev2body[2]);
        file.read_float(prefix+"C_dev2body[3]",p_C_dev2body[3]);
        file.read_float(prefix+"C_dev2body[4]",p_C_dev2body[4]);
        file.read_float(prefix+"C_dev2body[5]",p_C_dev2body[5]);
        file.read_float(prefix+"C_dev2body[6]",p_C_dev2body[6]);
        file.read_float(prefix+"C_dev2body[7]",p_C_dev2body[7]);
        file.read_float(prefix+"C_dev2body[8]",p_C_dev2body[8]);
      }
      void save_elems(const std::string& prefix, utilities::ofile& file) const {
        file.write_uint8(prefix+"op_mode",op_mode);
        file.write_uint8(prefix+"avg_samp",avg_samp);
        file.write_uint8(prefix+"rate",rate);
        file.write_uint8(prefix+"meas_mode",meas_mode);
        file.write_uint8(prefix+"fs_mag",fs_mag);
        file.write_uint32(prefix+"n_calib_samples",n_calib_samples);
        file.write_float(prefix+"C_dev2body[0]",p_C_dev2body[0]);
        file.write_float(prefix+"C_dev2body[1]",p_C_dev2body[1]);
        file.write_float(prefix+"C_dev2body[2]",p_C_dev2body[2]);
        file.write_float(prefix+"C_dev2body[3]",p_C_dev2body[3]);
        file.write_float(prefix+"C_dev2body[4]",p_C_dev2body[4]);
        file.write_float(prefix+"C_dev2body[5]",p_C_dev2body[5]);
        file.write_float(prefix+"C_dev2body[6]",p_C_dev2body[6]);
        file.write_float(prefix+"C_dev2body[7]",p_C_dev2body[7]);
        file.write_float(prefix+"C_dev2body[8]",p_C_dev2body[8]);
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class hmc5883l_config_ONBOARD : public hmc5883l_config<TOPIC_ONBOARD> {
      public:
        hmc5883l_config_ONBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        hmc5883l_config_ONBOARD(const hmc5883l_config_ONBOARD& rhs);
        hmc5883l_config_ONBOARD(uint8* ptr, uint32 offset);
        ~hmc5883l_config_ONBOARD();
    };
    class hmc5883l_config_OFFBOARD : public hmc5883l_config<TOPIC_OFFBOARD> {
      public:
        hmc5883l_config_OFFBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        hmc5883l_config_OFFBOARD(const hmc5883l_config_OFFBOARD& rhs);
        hmc5883l_config_OFFBOARD(uint8* ptr, uint32 offset);
        ~hmc5883l_config_OFFBOARD();
    };
    class hmc5883l_config_DEFAULT : public hmc5883l_config<TOPIC_DEFAULT> {
      public:
        hmc5883l_config_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        hmc5883l_config_DEFAULT(const hmc5883l_config_DEFAULT& rhs);
        hmc5883l_config_DEFAULT(uint8* ptr, uint32 offset);
        ~hmc5883l_config_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
