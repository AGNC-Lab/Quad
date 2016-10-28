////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/ahrs_data.cpp                                                                 //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "packets/ahrs_data.h"
using namespace packets;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  ahrs_data_AHRS::ahrs_data_AHRS(typename base::node_id_t node_id) : ahrs_data<TOPIC_AHRS>(node_id) {
    // Do nothing.
  }
  ahrs_data_AHRS::ahrs_data_AHRS(const ahrs_data_AHRS& rhs) : ahrs_data<TOPIC_AHRS>(rhs) {
    // Do nothing.
  }
  ahrs_data_AHRS::ahrs_data_AHRS(uint8* ptr, uint32 offset) : ahrs_data<TOPIC_AHRS>(ptr,offset) {
    // Do nothing.
  }
  ahrs_data_AHRS::~ahrs_data_AHRS() {
    // Do nothing.
  }

  ahrs_data_DEFAULT::ahrs_data_DEFAULT(typename base::node_id_t node_id) : ahrs_data<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  ahrs_data_DEFAULT::ahrs_data_DEFAULT(const ahrs_data_DEFAULT& rhs) : ahrs_data<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  ahrs_data_DEFAULT::ahrs_data_DEFAULT(uint8* ptr, uint32 offset) : ahrs_data<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  ahrs_data_DEFAULT::~ahrs_data_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
