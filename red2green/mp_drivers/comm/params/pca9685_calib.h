////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/pca9685_calib.h                                                                //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_PCA9685_CALIB_H
#define PARAMS_PCA9685_CALIB_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/ifile.h"
#include "utilities/ofile.h"
#include "utilities/params.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace params {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class pca9685_calib : public utilities::params<comm::meta,comm::meta::ID_PCA9685_CALIB,TOPIC,1*sizeof(float)> {
    public:
      typedef utilities::params<comm::meta,comm::meta::ID_PCA9685_CALIB,TOPIC,1*sizeof(float)> base;

    public:
      float& clock_correction_factor;

    public:
      pca9685_calib(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        clock_correction_factor(base::template bind< float >(0)) {
        assert(
          (TOPIC == comm::meta::ONBOARD) or
          (TOPIC == comm::meta::OFFBOARD) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      pca9685_calib(const pca9685_calib& rhs) :
        base(rhs),
        clock_correction_factor(base::template bind< float >(0)) {
        // Do nothing.
      }
      pca9685_calib(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        clock_correction_factor(base::template bind< float >(0)) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << clock_correction_factor;
        return stream;
      }
      pca9685_calib& operator=(const pca9685_calib& rhs) {
        base::operator=(rhs);
        return *this;
      }

      void load_elems(const std::string& prefix, const utilities::ifile& file) {
        file.read_float(prefix+"clock_correction_factor",clock_correction_factor);
      }
      void save_elems(const std::string& prefix, utilities::ofile& file) const {
        file.write_float(prefix+"clock_correction_factor",clock_correction_factor);
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class pca9685_calib_ONBOARD : public pca9685_calib<TOPIC_ONBOARD> {
      public:
        pca9685_calib_ONBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        pca9685_calib_ONBOARD(const pca9685_calib_ONBOARD& rhs);
        pca9685_calib_ONBOARD(uint8* ptr, uint32 offset);
        ~pca9685_calib_ONBOARD();
    };
    class pca9685_calib_OFFBOARD : public pca9685_calib<TOPIC_OFFBOARD> {
      public:
        pca9685_calib_OFFBOARD(typename base::node_id_t node_id=comm::meta::local_node_id());
        pca9685_calib_OFFBOARD(const pca9685_calib_OFFBOARD& rhs);
        pca9685_calib_OFFBOARD(uint8* ptr, uint32 offset);
        ~pca9685_calib_OFFBOARD();
    };
    class pca9685_calib_DEFAULT : public pca9685_calib<TOPIC_DEFAULT> {
      public:
        pca9685_calib_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        pca9685_calib_DEFAULT(const pca9685_calib_DEFAULT& rhs);
        pca9685_calib_DEFAULT(uint8* ptr, uint32 offset);
        ~pca9685_calib_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
