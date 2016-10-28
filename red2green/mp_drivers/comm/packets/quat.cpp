////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/quat.cpp                                                                      //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "packets/quat.h"
using namespace packets;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  quat_AHRS::quat_AHRS(typename base::node_id_t node_id) : quat<TOPIC_AHRS>(node_id) {
    // Do nothing.
  }
  quat_AHRS::quat_AHRS(const quat_AHRS& rhs) : quat<TOPIC_AHRS>(rhs) {
    // Do nothing.
  }
  quat_AHRS::quat_AHRS(uint8* ptr, uint32 offset) : quat<TOPIC_AHRS>(ptr,offset) {
    // Do nothing.
  }
  quat_AHRS::~quat_AHRS() {
    // Do nothing.
  }

  quat_QSIM::quat_QSIM(typename base::node_id_t node_id) : quat<TOPIC_QSIM>(node_id) {
    // Do nothing.
  }
  quat_QSIM::quat_QSIM(const quat_QSIM& rhs) : quat<TOPIC_QSIM>(rhs) {
    // Do nothing.
  }
  quat_QSIM::quat_QSIM(uint8* ptr, uint32 offset) : quat<TOPIC_QSIM>(ptr,offset) {
    // Do nothing.
  }
  quat_QSIM::~quat_QSIM() {
    // Do nothing.
  }

  quat_DEFAULT::quat_DEFAULT(typename base::node_id_t node_id) : quat<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  quat_DEFAULT::quat_DEFAULT(const quat_DEFAULT& rhs) : quat<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  quat_DEFAULT::quat_DEFAULT(uint8* ptr, uint32 offset) : quat<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  quat_DEFAULT::~quat_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
