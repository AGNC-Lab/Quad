////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/pca9685_calib.cpp                                                              //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "params/pca9685_calib.h"
using namespace params;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  pca9685_calib_ONBOARD::pca9685_calib_ONBOARD(typename base::node_id_t node_id) : pca9685_calib<TOPIC_ONBOARD>(node_id) {
    // Do nothing.
  }
  pca9685_calib_ONBOARD::pca9685_calib_ONBOARD(const pca9685_calib_ONBOARD& rhs) : pca9685_calib<TOPIC_ONBOARD>(rhs) {
    // Do nothing.
  }
  pca9685_calib_ONBOARD::pca9685_calib_ONBOARD(uint8* ptr, uint32 offset) : pca9685_calib<TOPIC_ONBOARD>(ptr,offset) {
    // Do nothing.
  }
  pca9685_calib_ONBOARD::~pca9685_calib_ONBOARD() {
    // Do nothing.
  }

  pca9685_calib_OFFBOARD::pca9685_calib_OFFBOARD(typename base::node_id_t node_id) : pca9685_calib<TOPIC_OFFBOARD>(node_id) {
    // Do nothing.
  }
  pca9685_calib_OFFBOARD::pca9685_calib_OFFBOARD(const pca9685_calib_OFFBOARD& rhs) : pca9685_calib<TOPIC_OFFBOARD>(rhs) {
    // Do nothing.
  }
  pca9685_calib_OFFBOARD::pca9685_calib_OFFBOARD(uint8* ptr, uint32 offset) : pca9685_calib<TOPIC_OFFBOARD>(ptr,offset) {
    // Do nothing.
  }
  pca9685_calib_OFFBOARD::~pca9685_calib_OFFBOARD() {
    // Do nothing.
  }

  pca9685_calib_DEFAULT::pca9685_calib_DEFAULT(typename base::node_id_t node_id) : pca9685_calib<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  pca9685_calib_DEFAULT::pca9685_calib_DEFAULT(const pca9685_calib_DEFAULT& rhs) : pca9685_calib<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  pca9685_calib_DEFAULT::pca9685_calib_DEFAULT(uint8* ptr, uint32 offset) : pca9685_calib<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  pca9685_calib_DEFAULT::~pca9685_calib_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
