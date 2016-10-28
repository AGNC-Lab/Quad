////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/mpu6050_calib.h                                                                //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_MPU6050_CALIB_H
#define PARAMS_MPU6050_CALIB_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/ifile.h"
#include "utilities/ofile.h"
#include "utilities/params.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace params {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class mpu6050_calib : public utilities::params<comm::meta,comm::meta::ID_MPU6050_CALIB,TOPIC,6*sizeof(float)> {
    public:
      typedef utilities::params<comm::meta,comm::meta::ID_MPU6050_CALIB,TOPIC,6*sizeof(float)> base;

    public:
      float* const p_accl_bias; Eigen::Map< Eigen::Matrix<float,3,1> > accl_bias;
      float* const p_gyro_bias; Eigen::Map< Eigen::Matrix<float,3,1> > gyro_bias;

    public:
      mpu6050_calib(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        p_accl_bias(&base::template bind< float >(0)), accl_bias(p_accl_bias),
        p_gyro_bias(&base::template bind< float >(3*sizeof(float))), gyro_bias(p_gyro_bias) {
        assert(
          (TOPIC == comm::meta::ONBOARD) or
          (TOPIC == comm::meta::OFFBOARD) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      mpu6050_calib(const mpu6050_calib& rhs) :
        base(rhs),
        p_accl_bias(&base::template bind< float >(0)), accl_bias(p_accl_bias),
        p_gyro_bias(&base::template bind< float >(3*sizeof(float))), gyro_bias(p_gyro_bias) {
        // Do nothing.
      }
      mpu6050_calib(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        p_accl_bias(&base::template bind< float >(0)), accl_bias(p_accl_bias),
        p_gyro_bias(&base::template bind< float >(3*sizeof(float))), gyro_bias(p_gyro_bias) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_accl_bias[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_accl_bias[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_accl_bias[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_gyro_bias[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_gyro_bias[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_gyro_bias[2];
        return stream;
      }
      mpu6050_calib& operator=(const mpu6050_calib& rhs) {
        base::operator=(rhs);
        return *this;
      }

      void load_elems(const std::string& prefix, const utilities::ifile& file) {
        file.read_float(prefix+"accl_bias[0]",p_accl_bias[0]);
        file.read_float(prefix+"accl_bias[1]",p_accl_bias[1]);
        file.read_float(prefix+"accl_bias[2]",p_accl_bias[2]);
        file.read_float(prefix+"gyro_bias[0]",p_gyro_bias[0]);
        file.read_float(prefix+"gyro_bias[1]",p_gyro_bias[1]);
        file.read_float(prefix+"gyro_bias[2]",p_gyro_bias[2]);
      }
      void save_elems(const std::string& prefix, utilities::ofile& file) const {
        file.write_float(prefix+"accl_bias[0]",p_accl_bias[0]);
        file.write_float(prefix+"accl_bias[1]",p_accl_bias[1]);
        file.write_float(prefix+"accl_bias[2]",p_accl_bias[2]);
        file.write_float(prefix+"gyro_bias[0]",p_gyro_bias[0]);
        file.write_float(prefix+"gyro_bias[1]",p_gyro_bias[1]);
        file.write_float(prefix+"gyro_bias[2]",p_gyro_bias[2]);
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class mpu6050_calib_ONBOARD : public mpu6050_calib<TOPIC_ONBOARD> {
      public:
        mpu6050_calib_ONBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        mpu6050_calib_ONBOARD(const mpu6050_calib_ONBOARD& rhs);
        mpu6050_calib_ONBOARD(uint8* ptr, uint32 offset);
        ~mpu6050_calib_ONBOARD();
    };
    class mpu6050_calib_OFFBOARD : public mpu6050_calib<TOPIC_OFFBOARD> {
      public:
        mpu6050_calib_OFFBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        mpu6050_calib_OFFBOARD(const mpu6050_calib_OFFBOARD& rhs);
        mpu6050_calib_OFFBOARD(uint8* ptr, uint32 offset);
        ~mpu6050_calib_OFFBOARD();
    };
    class mpu6050_calib_DEFAULT : public mpu6050_calib<TOPIC_DEFAULT> {
      public:
        mpu6050_calib_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        mpu6050_calib_DEFAULT(const mpu6050_calib_DEFAULT& rhs);
        mpu6050_calib_DEFAULT(uint8* ptr, uint32 offset);
        ~mpu6050_calib_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
