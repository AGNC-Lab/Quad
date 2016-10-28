////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   states/ahrs_state.cpp                                                                 //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "states/ahrs_state.h"
using namespace states;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  ahrs_state_DEFAULT::ahrs_state_DEFAULT(typename base::label_t s, typename base::node_id_t node_id) : ahrs_state::state<TOPIC_DEFAULT>(s,node_id) {
    // Do nothing.
  }
  ahrs_state_DEFAULT::ahrs_state_DEFAULT(const ahrs_state_DEFAULT& rhs) : ahrs_state::state<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  ahrs_state_DEFAULT::ahrs_state_DEFAULT(uint8* ptr, uint32 offset) : ahrs_state::state<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  ahrs_state_DEFAULT::~ahrs_state_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
