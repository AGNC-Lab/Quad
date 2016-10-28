////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   params/mp_ap_board_config.cpp                                                         //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "params/mp_ap_board_config.h"
using namespace params;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(PRECOMPILE_COMM_LIB)
  mp_ap_board_config_ONBOARD::mp_ap_board_config_ONBOARD(typename base::node_id_t node_id) : mp_ap_board_config<TOPIC_ONBOARD>(node_id) {
    // Do nothing.
  }
  mp_ap_board_config_ONBOARD::mp_ap_board_config_ONBOARD(const mp_ap_board_config_ONBOARD& rhs) : mp_ap_board_config<TOPIC_ONBOARD>(rhs) {
    // Do nothing.
  }
  mp_ap_board_config_ONBOARD::mp_ap_board_config_ONBOARD(uint8* ptr, uint32 offset) : mp_ap_board_config<TOPIC_ONBOARD>(ptr,offset) {
    // Do nothing.
  }
  mp_ap_board_config_ONBOARD::~mp_ap_board_config_ONBOARD() {
    // Do nothing.
  }

  mp_ap_board_config_OFFBOARD::mp_ap_board_config_OFFBOARD(typename base::node_id_t node_id) : mp_ap_board_config<TOPIC_OFFBOARD>(node_id) {
    // Do nothing.
  }
  mp_ap_board_config_OFFBOARD::mp_ap_board_config_OFFBOARD(const mp_ap_board_config_OFFBOARD& rhs) : mp_ap_board_config<TOPIC_OFFBOARD>(rhs) {
    // Do nothing.
  }
  mp_ap_board_config_OFFBOARD::mp_ap_board_config_OFFBOARD(uint8* ptr, uint32 offset) : mp_ap_board_config<TOPIC_OFFBOARD>(ptr,offset) {
    // Do nothing.
  }
  mp_ap_board_config_OFFBOARD::~mp_ap_board_config_OFFBOARD() {
    // Do nothing.
  }

  mp_ap_board_config_DEFAULT::mp_ap_board_config_DEFAULT(typename base::node_id_t node_id) : mp_ap_board_config<TOPIC_DEFAULT>(node_id) {
    // Do nothing.
  }
  mp_ap_board_config_DEFAULT::mp_ap_board_config_DEFAULT(const mp_ap_board_config_DEFAULT& rhs) : mp_ap_board_config<TOPIC_DEFAULT>(rhs) {
    // Do nothing.
  }
  mp_ap_board_config_DEFAULT::mp_ap_board_config_DEFAULT(uint8* ptr, uint32 offset) : mp_ap_board_config<TOPIC_DEFAULT>(ptr,offset) {
    // Do nothing.
  }
  mp_ap_board_config_DEFAULT::~mp_ap_board_config_DEFAULT() {
    // Do nothing.
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
