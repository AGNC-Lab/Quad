////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/pca9685_config.h                                                               //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_PCA9685_CONFIG_H
#define PARAMS_PCA9685_CONFIG_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/ifile.h"
#include "utilities/ofile.h"
#include "utilities/params.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace params {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class pca9685_config : public utilities::params<comm::meta,comm::meta::ID_PCA9685_CONFIG,TOPIC,sizeof(uint32)+21*sizeof(float)> {
    public:
      typedef utilities::params<comm::meta,comm::meta::ID_PCA9685_CONFIG,TOPIC,sizeof(uint32)+21*sizeof(float)> base;

    public:
      uint32& n_calib_samples;
      float& calibration_pulse_width_usec;
      float& calibration_update_rate;
      float& update_rate;
      float* const p_min_pw_usec; Eigen::Map< Eigen::Matrix<float,6,1> > min_pw_usec;
      float* const p_max_pw_usec; Eigen::Map< Eigen::Matrix<float,6,1> > max_pw_usec;
      float* const p_channel_offset; Eigen::Map< Eigen::Matrix<float,6,1> > channel_offset;

    public:
      pca9685_config(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        n_calib_samples(base::template bind< uint32 >(0)),
        calibration_pulse_width_usec(base::template bind< float >(sizeof(uint32)+0*sizeof(float))),
        calibration_update_rate(base::template bind< float >(sizeof(uint32)+1*sizeof(float))),
        update_rate(base::template bind< float >(sizeof(uint32)+2*sizeof(float))),
        p_min_pw_usec(&base::template bind< float >(sizeof(uint32)+3*sizeof(float))), min_pw_usec(p_min_pw_usec),
        p_max_pw_usec(&base::template bind< float >(sizeof(uint32)+9*sizeof(float))), max_pw_usec(p_max_pw_usec),
        p_channel_offset(&base::template bind< float >(sizeof(uint32)+15*sizeof(float))), channel_offset(p_channel_offset) {
        assert(
          (TOPIC == comm::meta::ONBOARD) or
          (TOPIC == comm::meta::OFFBOARD) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      pca9685_config(const pca9685_config& rhs) :
        base(rhs),
        n_calib_samples(base::template bind< uint32 >(0)),
        calibration_pulse_width_usec(base::template bind< float >(sizeof(uint32)+0*sizeof(float))),
        calibration_update_rate(base::template bind< float >(sizeof(uint32)+1*sizeof(float))),
        update_rate(base::template bind< float >(sizeof(uint32)+2*sizeof(float))),
        p_min_pw_usec(&base::template bind< float >(sizeof(uint32)+3*sizeof(float))), min_pw_usec(p_min_pw_usec),
        p_max_pw_usec(&base::template bind< float >(sizeof(uint32)+9*sizeof(float))), max_pw_usec(p_max_pw_usec),
        p_channel_offset(&base::template bind< float >(sizeof(uint32)+15*sizeof(float))), channel_offset(p_channel_offset) {
        // Do nothing.
      }
      pca9685_config(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        n_calib_samples(base::template bind< uint32 >(0)),
        calibration_pulse_width_usec(base::template bind< float >(sizeof(uint32)+0*sizeof(float))),
        calibration_update_rate(base::template bind< float >(sizeof(uint32)+1*sizeof(float))),
        update_rate(base::template bind< float >(sizeof(uint32)+2*sizeof(float))),
        p_min_pw_usec(&base::template bind< float >(sizeof(uint32)+3*sizeof(float))), min_pw_usec(p_min_pw_usec),
        p_max_pw_usec(&base::template bind< float >(sizeof(uint32)+9*sizeof(float))), max_pw_usec(p_max_pw_usec),
        p_channel_offset(&base::template bind< float >(sizeof(uint32)+15*sizeof(float))), channel_offset(p_channel_offset) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::fixed << std::noshowpos << std::setfill('0') << std::setw(utilities::serializable::WIDTH) << n_calib_samples << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << calibration_pulse_width_usec << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << calibration_update_rate << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << update_rate << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_min_pw_usec[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_min_pw_usec[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_min_pw_usec[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_min_pw_usec[3] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_min_pw_usec[4] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_min_pw_usec[5] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_max_pw_usec[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_max_pw_usec[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_max_pw_usec[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_max_pw_usec[3] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_max_pw_usec[4] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_max_pw_usec[5] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_channel_offset[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_channel_offset[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_channel_offset[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_channel_offset[3] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_channel_offset[4] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_channel_offset[5];
        return stream;
      }
      pca9685_config& operator=(const pca9685_config& rhs) {
        base::operator=(rhs);
        return *this;
      }

      void load_elems(const std::string& prefix, const utilities::ifile& file) {
        file.read_uint32(prefix+"n_calib_samples",n_calib_samples);
        file.read_float(prefix+"calibration_pulse_width_usec",calibration_pulse_width_usec);
        file.read_float(prefix+"calibration_update_rate",calibration_update_rate);
        file.read_float(prefix+"update_rate",update_rate);
        file.read_float(prefix+"min_pw_usec[0]",p_min_pw_usec[0]);
        file.read_float(prefix+"min_pw_usec[1]",p_min_pw_usec[1]);
        file.read_float(prefix+"min_pw_usec[2]",p_min_pw_usec[2]);
        file.read_float(prefix+"min_pw_usec[3]",p_min_pw_usec[3]);
        file.read_float(prefix+"min_pw_usec[4]",p_min_pw_usec[4]);
        file.read_float(prefix+"min_pw_usec[5]",p_min_pw_usec[5]);
        file.read_float(prefix+"max_pw_usec[0]",p_max_pw_usec[0]);
        file.read_float(prefix+"max_pw_usec[1]",p_max_pw_usec[1]);
        file.read_float(prefix+"max_pw_usec[2]",p_max_pw_usec[2]);
        file.read_float(prefix+"max_pw_usec[3]",p_max_pw_usec[3]);
        file.read_float(prefix+"max_pw_usec[4]",p_max_pw_usec[4]);
        file.read_float(prefix+"max_pw_usec[5]",p_max_pw_usec[5]);
        file.read_float(prefix+"channel_offset[0]",p_channel_offset[0]);
        file.read_float(prefix+"channel_offset[1]",p_channel_offset[1]);
        file.read_float(prefix+"channel_offset[2]",p_channel_offset[2]);
        file.read_float(prefix+"channel_offset[3]",p_channel_offset[3]);
        file.read_float(prefix+"channel_offset[4]",p_channel_offset[4]);
        file.read_float(prefix+"channel_offset[5]",p_channel_offset[5]);
      }
      void save_elems(const std::string& prefix, utilities::ofile& file) const {
        file.write_uint32(prefix+"n_calib_samples",n_calib_samples);
        file.write_float(prefix+"calibration_pulse_width_usec",calibration_pulse_width_usec);
        file.write_float(prefix+"calibration_update_rate",calibration_update_rate);
        file.write_float(prefix+"update_rate",update_rate);
        file.write_float(prefix+"min_pw_usec[0]",p_min_pw_usec[0]);
        file.write_float(prefix+"min_pw_usec[1]",p_min_pw_usec[1]);
        file.write_float(prefix+"min_pw_usec[2]",p_min_pw_usec[2]);
        file.write_float(prefix+"min_pw_usec[3]",p_min_pw_usec[3]);
        file.write_float(prefix+"min_pw_usec[4]",p_min_pw_usec[4]);
        file.write_float(prefix+"min_pw_usec[5]",p_min_pw_usec[5]);
        file.write_float(prefix+"max_pw_usec[0]",p_max_pw_usec[0]);
        file.write_float(prefix+"max_pw_usec[1]",p_max_pw_usec[1]);
        file.write_float(prefix+"max_pw_usec[2]",p_max_pw_usec[2]);
        file.write_float(prefix+"max_pw_usec[3]",p_max_pw_usec[3]);
        file.write_float(prefix+"max_pw_usec[4]",p_max_pw_usec[4]);
        file.write_float(prefix+"max_pw_usec[5]",p_max_pw_usec[5]);
        file.write_float(prefix+"channel_offset[0]",p_channel_offset[0]);
        file.write_float(prefix+"channel_offset[1]",p_channel_offset[1]);
        file.write_float(prefix+"channel_offset[2]",p_channel_offset[2]);
        file.write_float(prefix+"channel_offset[3]",p_channel_offset[3]);
        file.write_float(prefix+"channel_offset[4]",p_channel_offset[4]);
        file.write_float(prefix+"channel_offset[5]",p_channel_offset[5]);
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class pca9685_config_ONBOARD : public pca9685_config<TOPIC_ONBOARD> {
      public:
        pca9685_config_ONBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        pca9685_config_ONBOARD(const pca9685_config_ONBOARD& rhs);
        pca9685_config_ONBOARD(uint8* ptr, uint32 offset);
        ~pca9685_config_ONBOARD();
    };
    class pca9685_config_OFFBOARD : public pca9685_config<TOPIC_OFFBOARD> {
      public:
        pca9685_config_OFFBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        pca9685_config_OFFBOARD(const pca9685_config_OFFBOARD& rhs);
        pca9685_config_OFFBOARD(uint8* ptr, uint32 offset);
        ~pca9685_config_OFFBOARD();
    };
    class pca9685_config_DEFAULT : public pca9685_config<TOPIC_DEFAULT> {
      public:
        pca9685_config_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        pca9685_config_DEFAULT(const pca9685_config_DEFAULT& rhs);
        pca9685_config_DEFAULT(uint8* ptr, uint32 offset);
        ~pca9685_config_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
