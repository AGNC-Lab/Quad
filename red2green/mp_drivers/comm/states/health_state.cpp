////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   states/health_state.cpp                                                               //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "states/health_state.h"
using namespace states;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  health_state_HMC5883L::health_state_HMC5883L(typename base::label_t s, typename base::node_id_t node_id) : health_state::state<TOPIC_HMC5883L>(s,node_id) {
    // Do nothing.
  }
  health_state_HMC5883L::health_state_HMC5883L(const health_state_HMC5883L& rhs) : health_state::state<TOPIC_HMC5883L>(rhs) {
    // Do nothing.
  }
  health_state_HMC5883L::health_state_HMC5883L(uint8* ptr, uint32 offset) : health_state::state<TOPIC_HMC5883L>(ptr,offset) {
    // Do nothing.
  }
  health_state_HMC5883L::~health_state_HMC5883L() {
    // Do nothing.
  }

  health_state_MPU6050::health_state_MPU6050(typename base::label_t s, typename base::node_id_t node_id) : health_state::state<TOPIC_MPU6050>(s,node_id) {
    // Do nothing.
  }
  health_state_MPU6050::health_state_MPU6050(const health_state_MPU6050& rhs) : health_state::state<TOPIC_MPU6050>(rhs) {
    // Do nothing.
  }
  health_state_MPU6050::health_state_MPU6050(uint8* ptr, uint32 offset) : health_state::state<TOPIC_MPU6050>(ptr,offset) {
    // Do nothing.
  }
  health_state_MPU6050::~health_state_MPU6050() {
    // Do nothing.
  }

  health_state_DEFAULT::health_state_DEFAULT(typename base::label_t s, typename base::node_id_t node_id) : health_state::state<TOPIC_DEFAULT>(s,node_id) {
    // Do nothing.
  }
  health_state_DEFAULT::health_state_DEFAULT(const health_state_DEFAULT& rhs) : health_state::state<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  health_state_DEFAULT::health_state_DEFAULT(uint8* ptr, uint32 offset) : health_state::state<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  health_state_DEFAULT::~health_state_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
