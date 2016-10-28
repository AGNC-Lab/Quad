////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/pos_data.cpp                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "packets/pos_data.h"
using namespace packets;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  pos_data_MOCAP::pos_data_MOCAP(typename base::node_id_t node_id) : pos_data<TOPIC_MOCAP>(node_id) {
    // Do nothing.
  }
  pos_data_MOCAP::pos_data_MOCAP(const pos_data_MOCAP& rhs) : pos_data<TOPIC_MOCAP>(rhs) {
    // Do nothing.
  }
  pos_data_MOCAP::pos_data_MOCAP(uint8* ptr, uint32 offset) : pos_data<TOPIC_MOCAP>(ptr,offset) {
    // Do nothing.
  }
  pos_data_MOCAP::~pos_data_MOCAP() {
    // Do nothing.
  }

  pos_data_GPS::pos_data_GPS(typename base::node_id_t node_id) : pos_data<TOPIC_GPS>(node_id) {
    // Do nothing.
  }
  pos_data_GPS::pos_data_GPS(const pos_data_GPS& rhs) : pos_data<TOPIC_GPS>(rhs) {
    // Do nothing.
  }
  pos_data_GPS::pos_data_GPS(uint8* ptr, uint32 offset) : pos_data<TOPIC_GPS>(ptr,offset) {
    // Do nothing.
  }
  pos_data_GPS::~pos_data_GPS() {
    // Do nothing.
  }

  pos_data_SIM::pos_data_SIM(typename base::node_id_t node_id) : pos_data<TOPIC_SIM>(node_id) {
    // Do nothing.
  }
  pos_data_SIM::pos_data_SIM(const pos_data_SIM& rhs) : pos_data<TOPIC_SIM>(rhs) {
    // Do nothing.
  }
  pos_data_SIM::pos_data_SIM(uint8* ptr, uint32 offset) : pos_data<TOPIC_SIM>(ptr,offset) {
    // Do nothing.
  }
  pos_data_SIM::~pos_data_SIM() {
    // Do nothing.
  }

  pos_data_DEFAULT::pos_data_DEFAULT(typename base::node_id_t node_id) : pos_data<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  pos_data_DEFAULT::pos_data_DEFAULT(const pos_data_DEFAULT& rhs) : pos_data<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  pos_data_DEFAULT::pos_data_DEFAULT(uint8* ptr, uint32 offset) : pos_data<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  pos_data_DEFAULT::~pos_data_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
