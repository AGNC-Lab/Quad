////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   states/qctrl_state.cpp                                                                //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "states/qctrl_state.h"
using namespace states;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  qctrl_state_DEFAULT::qctrl_state_DEFAULT(typename base::label_t s, typename base::node_id_t node_id) : qctrl_state::state<TOPIC_DEFAULT>(s,node_id) {
    // Do nothing.
  }
  qctrl_state_DEFAULT::qctrl_state_DEFAULT(const qctrl_state_DEFAULT& rhs) : qctrl_state::state<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  qctrl_state_DEFAULT::qctrl_state_DEFAULT(uint8* ptr, uint32 offset) : qctrl_state::state<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  qctrl_state_DEFAULT::~qctrl_state_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
