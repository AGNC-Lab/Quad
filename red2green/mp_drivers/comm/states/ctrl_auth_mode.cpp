////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   states/ctrl_auth_mode.cpp                                                             //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "states/ctrl_auth_mode.h"
using namespace states;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  ctrl_auth_mode_DEFAULT::ctrl_auth_mode_DEFAULT(typename base::label_t s, typename base::node_id_t node_id) : ctrl_auth_mode::state<TOPIC_DEFAULT>(s,node_id) {
    // Do nothing.
  }
  ctrl_auth_mode_DEFAULT::ctrl_auth_mode_DEFAULT(const ctrl_auth_mode_DEFAULT& rhs) : ctrl_auth_mode::state<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  ctrl_auth_mode_DEFAULT::ctrl_auth_mode_DEFAULT(uint8* ptr, uint32 offset) : ctrl_auth_mode::state<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  ctrl_auth_mode_DEFAULT::~ctrl_auth_mode_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
