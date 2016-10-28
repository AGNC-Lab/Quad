////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/pwm_cmd.cpp                                                                   //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "packets/pwm_cmd.h"
using namespace packets;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  pwm_cmd_DEVICE::pwm_cmd_DEVICE(typename base::node_id_t node_id) : pwm_cmd<TOPIC_DEVICE>(node_id) {
    // Do nothing.
  }
  pwm_cmd_DEVICE::pwm_cmd_DEVICE(const pwm_cmd_DEVICE& rhs) : pwm_cmd<TOPIC_DEVICE>(rhs) {
    // Do nothing.
  }
  pwm_cmd_DEVICE::pwm_cmd_DEVICE(uint8* ptr, uint32 offset) : pwm_cmd<TOPIC_DEVICE>(ptr,offset) {
    // Do nothing.
  }
  pwm_cmd_DEVICE::~pwm_cmd_DEVICE() {
    // Do nothing.
  }

  pwm_cmd_SIM::pwm_cmd_SIM(typename base::node_id_t node_id) : pwm_cmd<TOPIC_SIM>(node_id) {
    // Do nothing.
  }
  pwm_cmd_SIM::pwm_cmd_SIM(const pwm_cmd_SIM& rhs) : pwm_cmd<TOPIC_SIM>(rhs) {
    // Do nothing.
  }
  pwm_cmd_SIM::pwm_cmd_SIM(uint8* ptr, uint32 offset) : pwm_cmd<TOPIC_SIM>(ptr,offset) {
    // Do nothing.
  }
  pwm_cmd_SIM::~pwm_cmd_SIM() {
    // Do nothing.
  }

  pwm_cmd_DEFAULT::pwm_cmd_DEFAULT(typename base::node_id_t node_id) : pwm_cmd<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  pwm_cmd_DEFAULT::pwm_cmd_DEFAULT(const pwm_cmd_DEFAULT& rhs) : pwm_cmd<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  pwm_cmd_DEFAULT::pwm_cmd_DEFAULT(uint8* ptr, uint32 offset) : pwm_cmd<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  pwm_cmd_DEFAULT::~pwm_cmd_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
