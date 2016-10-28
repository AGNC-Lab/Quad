////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   packets/qctrl_cmd.cpp                                                                 //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "packets/qctrl_cmd.h"
using namespace packets;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  qctrl_cmd_GCS::qctrl_cmd_GCS(typename base::node_id_t node_id) : qctrl_cmd<TOPIC_GCS>(node_id) {
    // Do nothing.
  }
  qctrl_cmd_GCS::qctrl_cmd_GCS(const qctrl_cmd_GCS& rhs) : qctrl_cmd<TOPIC_GCS>(rhs) {
    // Do nothing.
  }
  qctrl_cmd_GCS::qctrl_cmd_GCS(uint8* ptr, uint32 offset) : qctrl_cmd<TOPIC_GCS>(ptr,offset) {
    // Do nothing.
  }
  qctrl_cmd_GCS::~qctrl_cmd_GCS() {
    // Do nothing.
  }

  qctrl_cmd_API::qctrl_cmd_API(typename base::node_id_t node_id) : qctrl_cmd<TOPIC_API>(node_id) {
    // Do nothing.
  }
  qctrl_cmd_API::qctrl_cmd_API(const qctrl_cmd_API& rhs) : qctrl_cmd<TOPIC_API>(rhs) {
    // Do nothing.
  }
  qctrl_cmd_API::qctrl_cmd_API(uint8* ptr, uint32 offset) : qctrl_cmd<TOPIC_API>(ptr,offset) {
    // Do nothing.
  }
  qctrl_cmd_API::~qctrl_cmd_API() {
    // Do nothing.
  }

  qctrl_cmd_DEFAULT::qctrl_cmd_DEFAULT(typename base::node_id_t node_id) : qctrl_cmd<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  qctrl_cmd_DEFAULT::qctrl_cmd_DEFAULT(const qctrl_cmd_DEFAULT& rhs) : qctrl_cmd<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  qctrl_cmd_DEFAULT::qctrl_cmd_DEFAULT(uint8* ptr, uint32 offset) : qctrl_cmd<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  qctrl_cmd_DEFAULT::~qctrl_cmd_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
