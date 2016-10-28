////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/imu_data.cpp                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "packets/imu_data.h"
using namespace packets;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  imu_data_DEVICE::imu_data_DEVICE(typename base::node_id_t node_id) : imu_data<TOPIC_DEVICE>(node_id) {
    // Do nothing.
  }
  imu_data_DEVICE::imu_data_DEVICE(const imu_data_DEVICE& rhs) : imu_data<TOPIC_DEVICE>(rhs) {
    // Do nothing.
  }
  imu_data_DEVICE::imu_data_DEVICE(uint8* ptr, uint32 offset) : imu_data<TOPIC_DEVICE>(ptr,offset) {
    // Do nothing.
  }
  imu_data_DEVICE::~imu_data_DEVICE() {
    // Do nothing.
  }

  imu_data_SIM::imu_data_SIM(typename base::node_id_t node_id) : imu_data<TOPIC_SIM>(node_id) {
    // Do nothing.
  }
  imu_data_SIM::imu_data_SIM(const imu_data_SIM& rhs) : imu_data<TOPIC_SIM>(rhs) {
    // Do nothing.
  }
  imu_data_SIM::imu_data_SIM(uint8* ptr, uint32 offset) : imu_data<TOPIC_SIM>(ptr,offset) {
    // Do nothing.
  }
  imu_data_SIM::~imu_data_SIM() {
    // Do nothing.
  }

  imu_data_AHRS::imu_data_AHRS(typename base::node_id_t node_id) : imu_data<TOPIC_AHRS>(node_id) {
    // Do nothing.
  }
  imu_data_AHRS::imu_data_AHRS(const imu_data_AHRS& rhs) : imu_data<TOPIC_AHRS>(rhs) {
    // Do nothing.
  }
  imu_data_AHRS::imu_data_AHRS(uint8* ptr, uint32 offset) : imu_data<TOPIC_AHRS>(ptr,offset) {
    // Do nothing.
  }
  imu_data_AHRS::~imu_data_AHRS() {
    // Do nothing.
  }

  imu_data_DEFAULT::imu_data_DEFAULT(typename base::node_id_t node_id) : imu_data<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  imu_data_DEFAULT::imu_data_DEFAULT(const imu_data_DEFAULT& rhs) : imu_data<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  imu_data_DEFAULT::imu_data_DEFAULT(uint8* ptr, uint32 offset) : imu_data<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  imu_data_DEFAULT::~imu_data_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
