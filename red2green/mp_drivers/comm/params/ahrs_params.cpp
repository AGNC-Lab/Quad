////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/ahrs_params.cpp                                                                //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "params/ahrs_params.h"
using namespace params;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  ahrs_params_ONBOARD::ahrs_params_ONBOARD(typename base::node_id_t node_id) : ahrs_params<TOPIC_ONBOARD>(node_id) {
    // Do nothing.
  }
  ahrs_params_ONBOARD::ahrs_params_ONBOARD(const ahrs_params_ONBOARD& rhs) : ahrs_params<TOPIC_ONBOARD>(rhs) {
    // Do nothing.
  }
  ahrs_params_ONBOARD::ahrs_params_ONBOARD(uint8* ptr, uint32 offset) : ahrs_params<TOPIC_ONBOARD>(ptr,offset) {
    // Do nothing.
  }
  ahrs_params_ONBOARD::~ahrs_params_ONBOARD() {
    // Do nothing.
  }

  ahrs_params_OFFBOARD::ahrs_params_OFFBOARD(typename base::node_id_t node_id) : ahrs_params<TOPIC_OFFBOARD>(node_id) {
    // Do nothing.
  }
  ahrs_params_OFFBOARD::ahrs_params_OFFBOARD(const ahrs_params_OFFBOARD& rhs) : ahrs_params<TOPIC_OFFBOARD>(rhs) {
    // Do nothing.
  }
  ahrs_params_OFFBOARD::ahrs_params_OFFBOARD(uint8* ptr, uint32 offset) : ahrs_params<TOPIC_OFFBOARD>(ptr,offset) {
    // Do nothing.
  }
  ahrs_params_OFFBOARD::~ahrs_params_OFFBOARD() {
    // Do nothing.
  }

  ahrs_params_DEFAULT::ahrs_params_DEFAULT(typename base::node_id_t node_id) : ahrs_params<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  ahrs_params_DEFAULT::ahrs_params_DEFAULT(const ahrs_params_DEFAULT& rhs) : ahrs_params<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  ahrs_params_DEFAULT::ahrs_params_DEFAULT(uint8* ptr, uint32 offset) : ahrs_params<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  ahrs_params_DEFAULT::~ahrs_params_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
