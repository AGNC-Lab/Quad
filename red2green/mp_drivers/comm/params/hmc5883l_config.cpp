////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/hmc5883l_config.cpp                                                            //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "params/hmc5883l_config.h"
using namespace params;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  hmc5883l_config_ONBOARD::hmc5883l_config_ONBOARD(typename base::node_id_t node_id) : hmc5883l_config<TOPIC_ONBOARD>(node_id) {
    // Do nothing.
  }
  hmc5883l_config_ONBOARD::hmc5883l_config_ONBOARD(const hmc5883l_config_ONBOARD& rhs) : hmc5883l_config<TOPIC_ONBOARD>(rhs) {
    // Do nothing.
  }
  hmc5883l_config_ONBOARD::hmc5883l_config_ONBOARD(uint8* ptr, uint32 offset) : hmc5883l_config<TOPIC_ONBOARD>(ptr,offset) {
    // Do nothing.
  }
  hmc5883l_config_ONBOARD::~hmc5883l_config_ONBOARD() {
    // Do nothing.
  }

  hmc5883l_config_OFFBOARD::hmc5883l_config_OFFBOARD(typename base::node_id_t node_id) : hmc5883l_config<TOPIC_OFFBOARD>(node_id) {
    // Do nothing.
  }
  hmc5883l_config_OFFBOARD::hmc5883l_config_OFFBOARD(const hmc5883l_config_OFFBOARD& rhs) : hmc5883l_config<TOPIC_OFFBOARD>(rhs) {
    // Do nothing.
  }
  hmc5883l_config_OFFBOARD::hmc5883l_config_OFFBOARD(uint8* ptr, uint32 offset) : hmc5883l_config<TOPIC_OFFBOARD>(ptr,offset) {
    // Do nothing.
  }
  hmc5883l_config_OFFBOARD::~hmc5883l_config_OFFBOARD() {
    // Do nothing.
  }

  hmc5883l_config_DEFAULT::hmc5883l_config_DEFAULT(typename base::node_id_t node_id) : hmc5883l_config<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  hmc5883l_config_DEFAULT::hmc5883l_config_DEFAULT(const hmc5883l_config_DEFAULT& rhs) : hmc5883l_config<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  hmc5883l_config_DEFAULT::hmc5883l_config_DEFAULT(uint8* ptr, uint32 offset) : hmc5883l_config<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  hmc5883l_config_DEFAULT::~hmc5883l_config_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
