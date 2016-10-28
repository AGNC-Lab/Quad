////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/imu_data.h                                                                    //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PACKETS_IMU_DATA_H
#define PACKETS_IMU_DATA_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/packet.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace packets {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class imu_data : public utilities::packet<comm::meta,comm::meta::ID_IMU_DATA,TOPIC,6*sizeof(float)> {
    public:
      typedef utilities::packet<comm::meta,comm::meta::ID_IMU_DATA,TOPIC,6*sizeof(float)> base;

    public:
      float* const p_accl; Eigen::Map< Eigen::Matrix<float,3,1> > accl;
      float* const p_gyro; Eigen::Map< Eigen::Matrix<float,3,1> > gyro;

    public:
      imu_data(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        p_accl(&base::template bind< float >(0)), accl(p_accl),
        p_gyro(&base::template bind< float >(3*sizeof(float))), gyro(p_gyro) {
        assert(
          (TOPIC == comm::meta::DEVICE) or
          (TOPIC == comm::meta::SIM) or
          (TOPIC == comm::meta::AHRS) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      imu_data(const imu_data& rhs) :
        base(rhs),
        p_accl(&base::template bind< float >(0)), accl(p_accl),
        p_gyro(&base::template bind< float >(3*sizeof(float))), gyro(p_gyro) {
        // Do nothing.
      }
      imu_data(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        p_accl(&base::template bind< float >(0)), accl(p_accl),
        p_gyro(&base::template bind< float >(3*sizeof(float))), gyro(p_gyro) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_accl[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_accl[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_accl[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_gyro[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_gyro[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_gyro[2];
        return stream;
      }
      imu_data& operator=(const imu_data& rhs) {
        base::operator=(rhs);
        return *this;
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class imu_data_DEVICE : public imu_data<TOPIC_DEVICE> {
      public:
        imu_data_DEVICE(typename base::node_id_t node_id=comm::meta::local_node_id());
        imu_data_DEVICE(const imu_data_DEVICE& rhs);
        imu_data_DEVICE(uint8* ptr, uint32 offset);
        ~imu_data_DEVICE();
    };
    class imu_data_SIM : public imu_data<TOPIC_SIM> {
      public:
        imu_data_SIM(typename base::node_id_t node_id=comm::meta::local_node_id());
        imu_data_SIM(const imu_data_SIM& rhs);
        imu_data_SIM(uint8* ptr, uint32 offset);
        ~imu_data_SIM();
    };
    class imu_data_AHRS : public imu_data<TOPIC_AHRS> {
      public:
        imu_data_AHRS(typename base::node_id_t node_id=comm::meta::local_node_id());
        imu_data_AHRS(const imu_data_AHRS& rhs);
        imu_data_AHRS(uint8* ptr, uint32 offset);
        ~imu_data_AHRS();
    };
    class imu_data_DEFAULT : public imu_data<TOPIC_DEFAULT> {
      public:
        imu_data_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        imu_data_DEFAULT(const imu_data_DEFAULT& rhs);
        imu_data_DEFAULT(uint8* ptr, uint32 offset);
        ~imu_data_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
