////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/hmc5883l_calib.cpp                                                             //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "params/hmc5883l_calib.h"
using namespace params;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  hmc5883l_calib_ONBOARD::hmc5883l_calib_ONBOARD(typename base::node_id_t node_id) : hmc5883l_calib<TOPIC_ONBOARD>(node_id) {
    // Do nothing.
  }
  hmc5883l_calib_ONBOARD::hmc5883l_calib_ONBOARD(const hmc5883l_calib_ONBOARD& rhs) : hmc5883l_calib<TOPIC_ONBOARD>(rhs) {
    // Do nothing.
  }
  hmc5883l_calib_ONBOARD::hmc5883l_calib_ONBOARD(uint8* ptr, uint32 offset) : hmc5883l_calib<TOPIC_ONBOARD>(ptr,offset) {
    // Do nothing.
  }
  hmc5883l_calib_ONBOARD::~hmc5883l_calib_ONBOARD() {
    // Do nothing.
  }

  hmc5883l_calib_OFFBOARD::hmc5883l_calib_OFFBOARD(typename base::node_id_t node_id) : hmc5883l_calib<TOPIC_OFFBOARD>(node_id) {
    // Do nothing.
  }
  hmc5883l_calib_OFFBOARD::hmc5883l_calib_OFFBOARD(const hmc5883l_calib_OFFBOARD& rhs) : hmc5883l_calib<TOPIC_OFFBOARD>(rhs) {
    // Do nothing.
  }
  hmc5883l_calib_OFFBOARD::hmc5883l_calib_OFFBOARD(uint8* ptr, uint32 offset) : hmc5883l_calib<TOPIC_OFFBOARD>(ptr,offset) {
    // Do nothing.
  }
  hmc5883l_calib_OFFBOARD::~hmc5883l_calib_OFFBOARD() {
    // Do nothing.
  }

  hmc5883l_calib_DEFAULT::hmc5883l_calib_DEFAULT(typename base::node_id_t node_id) : hmc5883l_calib<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  hmc5883l_calib_DEFAULT::hmc5883l_calib_DEFAULT(const hmc5883l_calib_DEFAULT& rhs) : hmc5883l_calib<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  hmc5883l_calib_DEFAULT::hmc5883l_calib_DEFAULT(uint8* ptr, uint32 offset) : hmc5883l_calib<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  hmc5883l_calib_DEFAULT::~hmc5883l_calib_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
