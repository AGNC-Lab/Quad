////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/pid_gains.h                                                                    //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_PID_GAINS_H
#define PARAMS_PID_GAINS_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/ifile.h"
#include "utilities/ofile.h"
#include "utilities/params.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace params {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class pid_gains : public utilities::params<comm::meta,comm::meta::ID_PID_GAINS,TOPIC,5*sizeof(float)> {
    public:
      typedef utilities::params<comm::meta,comm::meta::ID_PID_GAINS,TOPIC,5*sizeof(float)> base;

    public:
      float& k_p;
      float& k_i;
      float& k_d;
      float& int_min;
      float& int_max;

    public:
      pid_gains(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        k_p(base::template bind< float >(0)),
        k_i(base::template bind< float >(1*sizeof(float))),
        k_d(base::template bind< float >(2*sizeof(float))),
        int_min(base::template bind< float >(3*sizeof(float))),
        int_max(base::template bind< float >(4*sizeof(float))) {
        assert(
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      pid_gains(const pid_gains& rhs) :
        base(rhs),
        k_p(base::template bind< float >(0)),
        k_i(base::template bind< float >(1*sizeof(float))),
        k_d(base::template bind< float >(2*sizeof(float))),
        int_min(base::template bind< float >(3*sizeof(float))),
        int_max(base::template bind< float >(4*sizeof(float))) {
        // Do nothing.
      }
      pid_gains(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        k_p(base::template bind< float >(0)),
        k_i(base::template bind< float >(1*sizeof(float))),
        k_d(base::template bind< float >(2*sizeof(float))),
        int_min(base::template bind< float >(3*sizeof(float))),
        int_max(base::template bind< float >(4*sizeof(float))) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << k_p << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << k_i << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << k_d << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << int_min << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << int_max;
        return stream;
      }
      pid_gains& operator=(const pid_gains& rhs) {
        base::operator=(rhs);
        return *this;
      }

      void load_elems(const std::string& prefix, const utilities::ifile& file) {
        file.read_float(prefix+"k_p",k_p);
        file.read_float(prefix+"k_i",k_i);
        file.read_float(prefix+"k_d",k_d);
        file.read_float(prefix+"int_min",int_min);
        file.read_float(prefix+"int_max",int_max);
      }
      void save_elems(const std::string& prefix, utilities::ofile& file) const {
        file.write_float(prefix+"k_p",k_p);
        file.write_float(prefix+"k_i",k_i);
        file.write_float(prefix+"k_d",k_d);
        file.write_float(prefix+"int_min",int_min);
        file.write_float(prefix+"int_max",int_max);
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class pid_gains_DEFAULT : public pid_gains<TOPIC_DEFAULT> {
      public:
        pid_gains_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        pid_gains_DEFAULT(const pid_gains_DEFAULT& rhs);
        pid_gains_DEFAULT(uint8* ptr, uint32 offset);
        ~pid_gains_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
