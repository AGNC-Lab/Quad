////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/mpu6050_calib.cpp                                                              //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "params/mpu6050_calib.h"
using namespace params;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  mpu6050_calib_ONBOARD::mpu6050_calib_ONBOARD(typename base::node_id_t node_id) : mpu6050_calib<TOPIC_ONBOARD>(node_id) {
    // Do nothing.
  }
  mpu6050_calib_ONBOARD::mpu6050_calib_ONBOARD(const mpu6050_calib_ONBOARD& rhs) : mpu6050_calib<TOPIC_ONBOARD>(rhs) {
    // Do nothing.
  }
  mpu6050_calib_ONBOARD::mpu6050_calib_ONBOARD(uint8* ptr, uint32 offset) : mpu6050_calib<TOPIC_ONBOARD>(ptr,offset) {
    // Do nothing.
  }
  mpu6050_calib_ONBOARD::~mpu6050_calib_ONBOARD() {
    // Do nothing.
  }

  mpu6050_calib_OFFBOARD::mpu6050_calib_OFFBOARD(typename base::node_id_t node_id) : mpu6050_calib<TOPIC_OFFBOARD>(node_id) {
    // Do nothing.
  }
  mpu6050_calib_OFFBOARD::mpu6050_calib_OFFBOARD(const mpu6050_calib_OFFBOARD& rhs) : mpu6050_calib<TOPIC_OFFBOARD>(rhs) {
    // Do nothing.
  }
  mpu6050_calib_OFFBOARD::mpu6050_calib_OFFBOARD(uint8* ptr, uint32 offset) : mpu6050_calib<TOPIC_OFFBOARD>(ptr,offset) {
    // Do nothing.
  }
  mpu6050_calib_OFFBOARD::~mpu6050_calib_OFFBOARD() {
    // Do nothing.
  }

  mpu6050_calib_DEFAULT::mpu6050_calib_DEFAULT(typename base::node_id_t node_id) : mpu6050_calib<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  mpu6050_calib_DEFAULT::mpu6050_calib_DEFAULT(const mpu6050_calib_DEFAULT& rhs) : mpu6050_calib<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  mpu6050_calib_DEFAULT::mpu6050_calib_DEFAULT(uint8* ptr, uint32 offset) : mpu6050_calib<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  mpu6050_calib_DEFAULT::~mpu6050_calib_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
