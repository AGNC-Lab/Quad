////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/mpu6050_config.cpp                                                             //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "params/mpu6050_config.h"
using namespace params;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  mpu6050_config_ONBOARD::mpu6050_config_ONBOARD(typename base::node_id_t node_id) : mpu6050_config<TOPIC_ONBOARD>(node_id) {
    // Do nothing.
  }
  mpu6050_config_ONBOARD::mpu6050_config_ONBOARD(const mpu6050_config_ONBOARD& rhs) : mpu6050_config<TOPIC_ONBOARD>(rhs) {
    // Do nothing.
  }
  mpu6050_config_ONBOARD::mpu6050_config_ONBOARD(uint8* ptr, uint32 offset) : mpu6050_config<TOPIC_ONBOARD>(ptr,offset) {
    // Do nothing.
  }
  mpu6050_config_ONBOARD::~mpu6050_config_ONBOARD() {
    // Do nothing.
  }

  mpu6050_config_OFFBOARD::mpu6050_config_OFFBOARD(typename base::node_id_t node_id) : mpu6050_config<TOPIC_OFFBOARD>(node_id) {
    // Do nothing.
  }
  mpu6050_config_OFFBOARD::mpu6050_config_OFFBOARD(const mpu6050_config_OFFBOARD& rhs) : mpu6050_config<TOPIC_OFFBOARD>(rhs) {
    // Do nothing.
  }
  mpu6050_config_OFFBOARD::mpu6050_config_OFFBOARD(uint8* ptr, uint32 offset) : mpu6050_config<TOPIC_OFFBOARD>(ptr,offset) {
    // Do nothing.
  }
  mpu6050_config_OFFBOARD::~mpu6050_config_OFFBOARD() {
    // Do nothing.
  }

  mpu6050_config_DEFAULT::mpu6050_config_DEFAULT(typename base::node_id_t node_id) : mpu6050_config<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  mpu6050_config_DEFAULT::mpu6050_config_DEFAULT(const mpu6050_config_DEFAULT& rhs) : mpu6050_config<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  mpu6050_config_DEFAULT::mpu6050_config_DEFAULT(uint8* ptr, uint32 offset) : mpu6050_config<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  mpu6050_config_DEFAULT::~mpu6050_config_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
