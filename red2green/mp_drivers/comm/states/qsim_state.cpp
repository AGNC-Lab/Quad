////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   states/qsim_state.cpp                                                                 //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "states/qsim_state.h"
using namespace states;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  qsim_state_DEFAULT::qsim_state_DEFAULT(typename base::label_t s, typename base::node_id_t node_id) : qsim_state::state<TOPIC_DEFAULT>(s,node_id) {
    // Do nothing.
  }
  qsim_state_DEFAULT::qsim_state_DEFAULT(const qsim_state_DEFAULT& rhs) : qsim_state::state<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  qsim_state_DEFAULT::qsim_state_DEFAULT(uint8* ptr, uint32 offset) : qsim_state::state<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  qsim_state_DEFAULT::~qsim_state_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
