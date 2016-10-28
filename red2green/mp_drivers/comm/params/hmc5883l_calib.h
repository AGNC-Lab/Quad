////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/hmc5883l_calib.h                                                               //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_HMC5883L_CALIB_H
#define PARAMS_HMC5883L_CALIB_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/ifile.h"
#include "utilities/ofile.h"
#include "utilities/params.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace params {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class hmc5883l_calib : public utilities::params<comm::meta,comm::meta::ID_HMC5883L_CALIB,TOPIC,3*sizeof(float)> {
    public:
      typedef utilities::params<comm::meta,comm::meta::ID_HMC5883L_CALIB,TOPIC,3*sizeof(float)> base;

    public:
      float* const p_mag_bias; Eigen::Map< Eigen::Matrix<float,3,1> > mag_bias;

    public:
      hmc5883l_calib(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        p_mag_bias(&base::template bind< float >(0)), mag_bias(p_mag_bias) {
        assert(
          (TOPIC == comm::meta::ONBOARD) or
          (TOPIC == comm::meta::OFFBOARD) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      hmc5883l_calib(const hmc5883l_calib& rhs) :
        base(rhs),
        p_mag_bias(&base::template bind< float >(0)), mag_bias(p_mag_bias) {
        // Do nothing.
      }
      hmc5883l_calib(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        p_mag_bias(&base::template bind< float >(0)), mag_bias(p_mag_bias) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_mag_bias[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_mag_bias[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_mag_bias[2];
        return stream;
      }
      hmc5883l_calib& operator=(const hmc5883l_calib& rhs) {
        base::operator=(rhs);
        return *this;
      }

      void load_elems(const std::string& prefix, const utilities::ifile& file) {
        file.read_float(prefix+"mag_bias[0]",p_mag_bias[0]);
        file.read_float(prefix+"mag_bias[1]",p_mag_bias[1]);
        file.read_float(prefix+"mag_bias[2]",p_mag_bias[2]);
      }
      void save_elems(const std::string& prefix, utilities::ofile& file) const {
        file.write_float(prefix+"mag_bias[0]",p_mag_bias[0]);
        file.write_float(prefix+"mag_bias[1]",p_mag_bias[1]);
        file.write_float(prefix+"mag_bias[2]",p_mag_bias[2]);
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class hmc5883l_calib_ONBOARD : public hmc5883l_calib<TOPIC_ONBOARD> {
      public:
        hmc5883l_calib_ONBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        hmc5883l_calib_ONBOARD(const hmc5883l_calib_ONBOARD& rhs);
        hmc5883l_calib_ONBOARD(uint8* ptr, uint32 offset);
        ~hmc5883l_calib_ONBOARD();
    };
    class hmc5883l_calib_OFFBOARD : public hmc5883l_calib<TOPIC_OFFBOARD> {
      public:
        hmc5883l_calib_OFFBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        hmc5883l_calib_OFFBOARD(const hmc5883l_calib_OFFBOARD& rhs);
        hmc5883l_calib_OFFBOARD(uint8* ptr, uint32 offset);
        ~hmc5883l_calib_OFFBOARD();
    };
    class hmc5883l_calib_DEFAULT : public hmc5883l_calib<TOPIC_DEFAULT> {
      public:
        hmc5883l_calib_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        hmc5883l_calib_DEFAULT(const hmc5883l_calib_DEFAULT& rhs);
        hmc5883l_calib_DEFAULT(uint8* ptr, uint32 offset);
        ~hmc5883l_calib_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
