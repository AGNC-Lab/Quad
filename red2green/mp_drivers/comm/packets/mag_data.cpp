////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/mag_data.cpp                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "packets/mag_data.h"
using namespace packets;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  mag_data_DEVICE::mag_data_DEVICE(typename base::node_id_t node_id) : mag_data<TOPIC_DEVICE>(node_id) {
    // Do nothing.
  }
  mag_data_DEVICE::mag_data_DEVICE(const mag_data_DEVICE& rhs) : mag_data<TOPIC_DEVICE>(rhs) {
    // Do nothing.
  }
  mag_data_DEVICE::mag_data_DEVICE(uint8* ptr, uint32 offset) : mag_data<TOPIC_DEVICE>(ptr,offset) {
    // Do nothing.
  }
  mag_data_DEVICE::~mag_data_DEVICE() {
    // Do nothing.
  }

  mag_data_SIM::mag_data_SIM(typename base::node_id_t node_id) : mag_data<TOPIC_SIM>(node_id) {
    // Do nothing.
  }
  mag_data_SIM::mag_data_SIM(const mag_data_SIM& rhs) : mag_data<TOPIC_SIM>(rhs) {
    // Do nothing.
  }
  mag_data_SIM::mag_data_SIM(uint8* ptr, uint32 offset) : mag_data<TOPIC_SIM>(ptr,offset) {
    // Do nothing.
  }
  mag_data_SIM::~mag_data_SIM() {
    // Do nothing.
  }

  mag_data_MOCAP::mag_data_MOCAP(typename base::node_id_t node_id) : mag_data<TOPIC_MOCAP>(node_id) {
    // Do nothing.
  }
  mag_data_MOCAP::mag_data_MOCAP(const mag_data_MOCAP& rhs) : mag_data<TOPIC_MOCAP>(rhs) {
    // Do nothing.
  }
  mag_data_MOCAP::mag_data_MOCAP(uint8* ptr, uint32 offset) : mag_data<TOPIC_MOCAP>(ptr,offset) {
    // Do nothing.
  }
  mag_data_MOCAP::~mag_data_MOCAP() {
    // Do nothing.
  }

  mag_data_DEFAULT::mag_data_DEFAULT(typename base::node_id_t node_id) : mag_data<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  mag_data_DEFAULT::mag_data_DEFAULT(const mag_data_DEFAULT& rhs) : mag_data<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  mag_data_DEFAULT::mag_data_DEFAULT(uint8* ptr, uint32 offset) : mag_data<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  mag_data_DEFAULT::~mag_data_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
