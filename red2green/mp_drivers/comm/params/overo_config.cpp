////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/overo_config.cpp                                                               //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "params/overo_config.h"
using namespace params;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  overo_config_ONBOARD::overo_config_ONBOARD(typename base::node_id_t node_id) : overo_config<TOPIC_ONBOARD>(node_id) {
    // Do nothing.
  }
  overo_config_ONBOARD::overo_config_ONBOARD(const overo_config_ONBOARD& rhs) : overo_config<TOPIC_ONBOARD>(rhs) {
    // Do nothing.
  }
  overo_config_ONBOARD::overo_config_ONBOARD(uint8* ptr, uint32 offset) : overo_config<TOPIC_ONBOARD>(ptr,offset) {
    // Do nothing.
  }
  overo_config_ONBOARD::~overo_config_ONBOARD() {
    // Do nothing.
  }

  overo_config_OFFBOARD::overo_config_OFFBOARD(typename base::node_id_t node_id) : overo_config<TOPIC_OFFBOARD>(node_id) {
    // Do nothing.
  }
  overo_config_OFFBOARD::overo_config_OFFBOARD(const overo_config_OFFBOARD& rhs) : overo_config<TOPIC_OFFBOARD>(rhs) {
    // Do nothing.
  }
  overo_config_OFFBOARD::overo_config_OFFBOARD(uint8* ptr, uint32 offset) : overo_config<TOPIC_OFFBOARD>(ptr,offset) {
    // Do nothing.
  }
  overo_config_OFFBOARD::~overo_config_OFFBOARD() {
    // Do nothing.
  }

  overo_config_DEFAULT::overo_config_DEFAULT(typename base::node_id_t node_id) : overo_config<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  overo_config_DEFAULT::overo_config_DEFAULT(const overo_config_DEFAULT& rhs) : overo_config<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  overo_config_DEFAULT::overo_config_DEFAULT(uint8* ptr, uint32 offset) : overo_config<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  overo_config_DEFAULT::~overo_config_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
