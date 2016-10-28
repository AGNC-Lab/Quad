////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/mpu6050_config.h                                                               //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_MPU6050_CONFIG_H
#define PARAMS_MPU6050_CONFIG_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/ifile.h"
#include "utilities/ofile.h"
#include "utilities/params.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace params {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class mpu6050_config : public utilities::params<comm::meta,comm::meta::ID_MPU6050_CONFIG,TOPIC,sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint32)+9*sizeof(float)> {
    public:
      typedef utilities::params<comm::meta,comm::meta::ID_MPU6050_CONFIG,TOPIC,sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint32)+9*sizeof(float)> base;

    public:
      uint8& dlpf_cfg;
      uint8& sample_rate_div;
      uint8& accl_fs;
      uint8& gyro_fs;
      uint32& n_calib_samples;
      float* const p_C_dev2body; Eigen::Map< Eigen::Matrix<float,3,3> > C_dev2body;

    public:
      mpu6050_config(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        dlpf_cfg(base::template bind< uint8 >(0)),
        sample_rate_div(base::template bind< uint8 >(sizeof(uint8)+0*sizeof(float))),
        accl_fs(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        gyro_fs(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        n_calib_samples(base::template bind< uint32 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        p_C_dev2body(&base::template bind< float >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint32)+0*sizeof(float))), C_dev2body(p_C_dev2body) {
        assert(
          (TOPIC == comm::meta::ONBOARD) or
          (TOPIC == comm::meta::OFFBOARD) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      mpu6050_config(const mpu6050_config& rhs) :
        base(rhs),
        dlpf_cfg(base::template bind< uint8 >(0)),
        sample_rate_div(base::template bind< uint8 >(sizeof(uint8)+0*sizeof(float))),
        accl_fs(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        gyro_fs(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        n_calib_samples(base::template bind< uint32 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        p_C_dev2body(&base::template bind< float >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint32)+0*sizeof(float))), C_dev2body(p_C_dev2body) {
        // Do nothing.
      }
      mpu6050_config(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        dlpf_cfg(base::template bind< uint8 >(0)),
        sample_rate_div(base::template bind< uint8 >(sizeof(uint8)+0*sizeof(float))),
        accl_fs(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        gyro_fs(base::template bind< uint8 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        n_calib_samples(base::template bind< uint32 >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+0*sizeof(float))),
        p_C_dev2body(&base::template bind< float >(sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint8)+sizeof(uint32)+0*sizeof(float))), C_dev2body(p_C_dev2body) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << dlpf_cfg << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << sample_rate_div << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << accl_fs << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << gyro_fs << utilities::serializable::DELIMITER;
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
      mpu6050_config& operator=(const mpu6050_config& rhs) {
        base::operator=(rhs);
        return *this;
      }

      void load_elems(const std::string& prefix, const utilities::ifile& file) {
        file.read_uint8(prefix+"dlpf_cfg",dlpf_cfg);
        file.read_uint8(prefix+"sample_rate_div",sample_rate_div);
        file.read_uint8(prefix+"accl_fs",accl_fs);
        file.read_uint8(prefix+"gyro_fs",gyro_fs);
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
        file.write_uint8(prefix+"dlpf_cfg",dlpf_cfg);
        file.write_uint8(prefix+"sample_rate_div",sample_rate_div);
        file.write_uint8(prefix+"accl_fs",accl_fs);
        file.write_uint8(prefix+"gyro_fs",gyro_fs);
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
    class mpu6050_config_ONBOARD : public mpu6050_config<TOPIC_ONBOARD> {
      public:
        mpu6050_config_ONBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        mpu6050_config_ONBOARD(const mpu6050_config_ONBOARD& rhs);
        mpu6050_config_ONBOARD(uint8* ptr, uint32 offset);
        ~mpu6050_config_ONBOARD();
    };
    class mpu6050_config_OFFBOARD : public mpu6050_config<TOPIC_OFFBOARD> {
      public:
        mpu6050_config_OFFBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        mpu6050_config_OFFBOARD(const mpu6050_config_OFFBOARD& rhs);
        mpu6050_config_OFFBOARD(uint8* ptr, uint32 offset);
        ~mpu6050_config_OFFBOARD();
    };
    class mpu6050_config_DEFAULT : public mpu6050_config<TOPIC_DEFAULT> {
      public:
        mpu6050_config_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        mpu6050_config_DEFAULT(const mpu6050_config_DEFAULT& rhs);
        mpu6050_config_DEFAULT(uint8* ptr, uint32 offset);
        ~mpu6050_config_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
