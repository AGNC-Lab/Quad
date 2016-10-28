////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   states/pwm_state.cpp                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "states/pwm_state.h"
using namespace states;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  pwm_state_DEFAULT::pwm_state_DEFAULT(typename base::label_t s, typename base::node_id_t node_id) : pwm_state::state<TOPIC_DEFAULT>(s,node_id) {
    // Do nothing.
  }
  pwm_state_DEFAULT::pwm_state_DEFAULT(const pwm_state_DEFAULT& rhs) : pwm_state::state<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  pwm_state_DEFAULT::pwm_state_DEFAULT(uint8* ptr, uint32 offset) : pwm_state::state<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  pwm_state_DEFAULT::~pwm_state_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
