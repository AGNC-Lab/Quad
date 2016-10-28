////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/qctrl_params.cpp                                                               //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "params/qctrl_params.h"
using namespace params;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  qctrl_params_ONBOARD::qctrl_params_ONBOARD(typename base::node_id_t node_id) : qctrl_params<TOPIC_ONBOARD>(node_id) {
    // Do nothing.
  }
  qctrl_params_ONBOARD::qctrl_params_ONBOARD(const qctrl_params_ONBOARD& rhs) : qctrl_params<TOPIC_ONBOARD>(rhs) {
    // Do nothing.
  }
  qctrl_params_ONBOARD::qctrl_params_ONBOARD(uint8* ptr, uint32 offset) : qctrl_params<TOPIC_ONBOARD>(ptr,offset) {
    // Do nothing.
  }
  qctrl_params_ONBOARD::~qctrl_params_ONBOARD() {
    // Do nothing.
  }

  qctrl_params_OFFBOARD::qctrl_params_OFFBOARD(typename base::node_id_t node_id) : qctrl_params<TOPIC_OFFBOARD>(node_id) {
    // Do nothing.
  }
  qctrl_params_OFFBOARD::qctrl_params_OFFBOARD(const qctrl_params_OFFBOARD& rhs) : qctrl_params<TOPIC_OFFBOARD>(rhs) {
    // Do nothing.
  }
  qctrl_params_OFFBOARD::qctrl_params_OFFBOARD(uint8* ptr, uint32 offset) : qctrl_params<TOPIC_OFFBOARD>(ptr,offset) {
    // Do nothing.
  }
  qctrl_params_OFFBOARD::~qctrl_params_OFFBOARD() {
    // Do nothing.
  }

  qctrl_params_DEFAULT::qctrl_params_DEFAULT(typename base::node_id_t node_id) : qctrl_params<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  qctrl_params_DEFAULT::qctrl_params_DEFAULT(const qctrl_params_DEFAULT& rhs) : qctrl_params<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  qctrl_params_DEFAULT::qctrl_params_DEFAULT(uint8* ptr, uint32 offset) : qctrl_params<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  qctrl_params_DEFAULT::~qctrl_params_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
