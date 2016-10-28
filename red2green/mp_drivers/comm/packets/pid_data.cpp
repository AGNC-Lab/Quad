////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/pid_data.cpp                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "packets/pid_data.h"
using namespace packets;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  pid_data_DEFAULT::pid_data_DEFAULT(typename base::node_id_t node_id) : pid_data<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  pid_data_DEFAULT::pid_data_DEFAULT(const pid_data_DEFAULT& rhs) : pid_data<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  pid_data_DEFAULT::pid_data_DEFAULT(uint8* ptr, uint32 offset) : pid_data<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  pid_data_DEFAULT::~pid_data_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
