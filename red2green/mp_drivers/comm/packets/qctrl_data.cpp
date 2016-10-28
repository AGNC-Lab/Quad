////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/qctrl_data.cpp                                                                //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "packets/qctrl_data.h"
using namespace packets;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  qctrl_data_QCTRL::qctrl_data_QCTRL(typename base::node_id_t node_id) : qctrl_data<TOPIC_QCTRL>(node_id) {
    // Do nothing.
  }
  qctrl_data_QCTRL::qctrl_data_QCTRL(const qctrl_data_QCTRL& rhs) : qctrl_data<TOPIC_QCTRL>(rhs) {
    // Do nothing.
  }
  qctrl_data_QCTRL::qctrl_data_QCTRL(uint8* ptr, uint32 offset) : qctrl_data<TOPIC_QCTRL>(ptr,offset) {
    // Do nothing.
  }
  qctrl_data_QCTRL::~qctrl_data_QCTRL() {
    // Do nothing.
  }

  qctrl_data_DEFAULT::qctrl_data_DEFAULT(typename base::node_id_t node_id) : qctrl_data<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  qctrl_data_DEFAULT::qctrl_data_DEFAULT(const qctrl_data_DEFAULT& rhs) : qctrl_data<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  qctrl_data_DEFAULT::qctrl_data_DEFAULT(uint8* ptr, uint32 offset) : qctrl_data<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  qctrl_data_DEFAULT::~qctrl_data_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
