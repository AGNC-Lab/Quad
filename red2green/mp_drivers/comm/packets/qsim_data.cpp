////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/qsim_data.cpp                                                                 //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "packets/qsim_data.h"
using namespace packets;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  qsim_data_QSIM::qsim_data_QSIM(typename base::node_id_t node_id) : qsim_data<TOPIC_QSIM>(node_id) {
    // Do nothing.
  }
  qsim_data_QSIM::qsim_data_QSIM(const qsim_data_QSIM& rhs) : qsim_data<TOPIC_QSIM>(rhs) {
    // Do nothing.
  }
  qsim_data_QSIM::qsim_data_QSIM(uint8* ptr, uint32 offset) : qsim_data<TOPIC_QSIM>(ptr,offset) {
    // Do nothing.
  }
  qsim_data_QSIM::~qsim_data_QSIM() {
    // Do nothing.
  }

  qsim_data_DEFAULT::qsim_data_DEFAULT(typename base::node_id_t node_id) : qsim_data<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  qsim_data_DEFAULT::qsim_data_DEFAULT(const qsim_data_DEFAULT& rhs) : qsim_data<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  qsim_data_DEFAULT::qsim_data_DEFAULT(uint8* ptr, uint32 offset) : qsim_data<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  qsim_data_DEFAULT::~qsim_data_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
