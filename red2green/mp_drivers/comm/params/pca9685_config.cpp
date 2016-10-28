////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/pca9685_config.cpp                                                             //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "params/pca9685_config.h"
using namespace params;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  pca9685_config_ONBOARD::pca9685_config_ONBOARD(typename base::node_id_t node_id) : pca9685_config<TOPIC_ONBOARD>(node_id) {
    // Do nothing.
  }
  pca9685_config_ONBOARD::pca9685_config_ONBOARD(const pca9685_config_ONBOARD& rhs) : pca9685_config<TOPIC_ONBOARD>(rhs) {
    // Do nothing.
  }
  pca9685_config_ONBOARD::pca9685_config_ONBOARD(uint8* ptr, uint32 offset) : pca9685_config<TOPIC_ONBOARD>(ptr,offset) {
    // Do nothing.
  }
  pca9685_config_ONBOARD::~pca9685_config_ONBOARD() {
    // Do nothing.
  }

  pca9685_config_OFFBOARD::pca9685_config_OFFBOARD(typename base::node_id_t node_id) : pca9685_config<TOPIC_OFFBOARD>(node_id) {
    // Do nothing.
  }
  pca9685_config_OFFBOARD::pca9685_config_OFFBOARD(const pca9685_config_OFFBOARD& rhs) : pca9685_config<TOPIC_OFFBOARD>(rhs) {
    // Do nothing.
  }
  pca9685_config_OFFBOARD::pca9685_config_OFFBOARD(uint8* ptr, uint32 offset) : pca9685_config<TOPIC_OFFBOARD>(ptr,offset) {
    // Do nothing.
  }
  pca9685_config_OFFBOARD::~pca9685_config_OFFBOARD() {
    // Do nothing.
  }

  pca9685_config_DEFAULT::pca9685_config_DEFAULT(typename base::node_id_t node_id) : pca9685_config<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  pca9685_config_DEFAULT::pca9685_config_DEFAULT(const pca9685_config_DEFAULT& rhs) : pca9685_config<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  pca9685_config_DEFAULT::pca9685_config_DEFAULT(uint8* ptr, uint32 offset) : pca9685_config<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  pca9685_config_DEFAULT::~pca9685_config_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
