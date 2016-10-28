////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   states/mag_state.cpp                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "states/mag_state.h"
using namespace states;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  mag_state_DEFAULT::mag_state_DEFAULT(typename base::label_t s, typename base::node_id_t node_id) : mag_state::state<TOPIC_DEFAULT>(s,node_id) {
    // Do nothing.
  }
  mag_state_DEFAULT::mag_state_DEFAULT(const mag_state_DEFAULT& rhs) : mag_state::state<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  mag_state_DEFAULT::mag_state_DEFAULT(uint8* ptr, uint32 offset) : mag_state::state<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  mag_state_DEFAULT::~mag_state_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
