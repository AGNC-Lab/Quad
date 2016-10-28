////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/qsim_data.h                                                                   //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PACKETS_QSIM_DATA_H
#define PACKETS_QSIM_DATA_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/packet.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace packets {
  template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
  class qsim_data : public utilities::packet<comm::meta,comm::meta::ID_QSIM_DATA,TOPIC,17*sizeof(double)+2*sizeof(float)> {
    public:
      typedef utilities::packet<comm::meta,comm::meta::ID_QSIM_DATA,TOPIC,17*sizeof(double)+2*sizeof(float)> base;

    public:
      float& ts_last;
      float& dt;
      double* const p_x; Eigen::Map< Eigen::Matrix<double,17,1> > x;

    public:
      qsim_data(typename base::node_id_t node_id=comm::meta::local_node_id()) :
        base(node_id),
        ts_last(base::template bind< float >(0)),
        dt(base::template bind< float >(1*sizeof(float))),
        p_x(&base::template bind< double >(2*sizeof(float))), x(p_x) {
        assert(
          (TOPIC == comm::meta::QSIM) or
          (TOPIC == comm::meta::DEFAULT)
        );
      }
      qsim_data(const qsim_data& rhs) :
        base(rhs),
        ts_last(base::template bind< float >(0)),
        dt(base::template bind< float >(1*sizeof(float))),
        p_x(&base::template bind< double >(2*sizeof(float))), x(p_x) {
        // Do nothing.
      }
      qsim_data(uint8* ptr, uint32 offset) :
        base(ptr,offset),
        ts_last(base::template bind< float >(0)),
        dt(base::template bind< float >(1*sizeof(float))),
        p_x(&base::template bind< double >(2*sizeof(float))), x(p_x) {
        // Do nothing.
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << ts_last << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << dt << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[0] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[1] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[2] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[3] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[4] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[5] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[6] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[7] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[8] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[9] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[10] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[11] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[12] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[13] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[14] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[15] << utilities::serializable::DELIMITER;
        stream << std::setbase(10) << std::scientific << std::showpos << std::setfill('0') << std::setprecision(utilities::serializable::PRECISION) << std::setw(utilities::serializable::PRECISION+3) << p_x[16];
        return stream;
      }
      qsim_data& operator=(const qsim_data& rhs) {
        base::operator=(rhs);
        return *this;
      }
  };

  #if defined(PRECOMPILE_COMM_LIB)
    class qsim_data_QSIM : public qsim_data<TOPIC_QSIM> {
      public:
        qsim_data_QSIM(typename base::node_id_t node_id=comm::meta::local_node_id());
        qsim_data_QSIM(const qsim_data_QSIM& rhs);
        qsim_data_QSIM(uint8* ptr, uint32 offset);
        ~qsim_data_QSIM();
    };
    class qsim_data_DEFAULT : public qsim_data<TOPIC_DEFAULT> {
      public:
        qsim_data_DEFAULT(typename base::node_id_t node_id=comm::meta::local_node_id());
        qsim_data_DEFAULT(const qsim_data_DEFAULT& rhs);
        qsim_data_DEFAULT(uint8* ptr, uint32 offset);
        ~qsim_data_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
