////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   states/qctrl_mode.cpp                                                                 //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "states/qctrl_mode.h"
using namespace states;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  qctrl_mode_DEFAULT::qctrl_mode_DEFAULT(typename base::label_t s, typename base::node_id_t node_id) : qctrl_mode::state<TOPIC_DEFAULT>(s,node_id) {
    // Do nothing.
  }
  qctrl_mode_DEFAULT::qctrl_mode_DEFAULT(const qctrl_mode_DEFAULT& rhs) : qctrl_mode::state<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  qctrl_mode_DEFAULT::qctrl_mode_DEFAULT(uint8* ptr, uint32 offset) : qctrl_mode::state<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  qctrl_mode_DEFAULT::~qctrl_mode_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
