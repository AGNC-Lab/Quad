////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/qsim_params.cpp                                                                //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "params/qsim_params.h"
using namespace params;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  qsim_params_ONBOARD::qsim_params_ONBOARD(typename base::node_id_t node_id) : qsim_params<TOPIC_ONBOARD>(node_id) {
    // Do nothing.
  }
  qsim_params_ONBOARD::qsim_params_ONBOARD(const qsim_params_ONBOARD& rhs) : qsim_params<TOPIC_ONBOARD>(rhs) {
    // Do nothing.
  }
  qsim_params_ONBOARD::qsim_params_ONBOARD(uint8* ptr, uint32 offset) : qsim_params<TOPIC_ONBOARD>(ptr,offset) {
    // Do nothing.
  }
  qsim_params_ONBOARD::~qsim_params_ONBOARD() {
    // Do nothing.
  }

  qsim_params_OFFBOARD::qsim_params_OFFBOARD(typename base::node_id_t node_id) : qsim_params<TOPIC_OFFBOARD>(node_id) {
    // Do nothing.
  }
  qsim_params_OFFBOARD::qsim_params_OFFBOARD(const qsim_params_OFFBOARD& rhs) : qsim_params<TOPIC_OFFBOARD>(rhs) {
    // Do nothing.
  }
  qsim_params_OFFBOARD::qsim_params_OFFBOARD(uint8* ptr, uint32 offset) : qsim_params<TOPIC_OFFBOARD>(ptr,offset) {
    // Do nothing.
  }
  qsim_params_OFFBOARD::~qsim_params_OFFBOARD() {
    // Do nothing.
  }

  qsim_params_DEFAULT::qsim_params_DEFAULT(typename base::node_id_t node_id) : qsim_params<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  qsim_params_DEFAULT::qsim_params_DEFAULT(const qsim_params_DEFAULT& rhs) : qsim_params<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  qsim_params_DEFAULT::qsim_params_DEFAULT(uint8* ptr, uint32 offset) : qsim_params<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  qsim_params_DEFAULT::~qsim_params_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
