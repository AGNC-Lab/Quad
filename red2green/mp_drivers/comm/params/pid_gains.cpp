////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/pid_gains.cpp                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "params/pid_gains.h"
using namespace params;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  pid_gains_DEFAULT::pid_gains_DEFAULT(typename base::node_id_t node_id) : pid_gains<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  pid_gains_DEFAULT::pid_gains_DEFAULT(const pid_gains_DEFAULT& rhs) : pid_gains<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  pid_gains_DEFAULT::pid_gains_DEFAULT(uint8* ptr, uint32 offset) : pid_gains<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  pid_gains_DEFAULT::~pid_gains_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
