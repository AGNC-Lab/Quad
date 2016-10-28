////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   states/imu_state.cpp                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "states/imu_state.h"
using namespace states;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  imu_state_DEFAULT::imu_state_DEFAULT(typename base::label_t s, typename base::node_id_t node_id) : imu_state::state<TOPIC_DEFAULT>(s,node_id) {
    // Do nothing.
  }
  imu_state_DEFAULT::imu_state_DEFAULT(const imu_state_DEFAULT& rhs) : imu_state::state<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  imu_state_DEFAULT::imu_state_DEFAULT(uint8* ptr, uint32 offset) : imu_state::state<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  imu_state_DEFAULT::~imu_state_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
