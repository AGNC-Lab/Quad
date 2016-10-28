////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   states/qctrl_mode.h                                                                   //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STATES_QCTRL_MODE_H
#define STATES_QCTRL_MODE_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/state.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace states {
  namespace qctrl_mode {
    enum label_t {
      STANDBY_MODE,
      MOTOR_TEST_MODE,
      MOTOR_CTRL_MODE,
      INERTIAL_CTRL_MODE_1,
      INERTIAL_CTRL_MODE_2,
      INERTIAL_CTRL_MODE_3,
      TILT_COMP_MODE,
      ALT_HOLD_MODE,
      VEL_CTRL_MODE,
      PVA_CTRL_MODE,
      BEGIN=STANDBY_MODE,
      END=PVA_CTRL_MODE,
      DEFAULT=STANDBY_MODE
    };

    template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
    class state : public utilities::state<comm::meta,comm::meta::ID_QCTRL_MODE,TOPIC,label_t> {
      public:
        typedef utilities::state<comm::meta,comm::meta::ID_QCTRL_MODE,TOPIC,label_t> base;

      public:
        state(typename base::label_t s=DEFAULT,
              typename base::node_id_t n=comm::meta::local_node_id()) :
          base(s,n) {
          assert(
            (TOPIC == comm::meta::DEFAULT)
          );
        }
        state(const state& rhs) :
          base(rhs) {
          // Do nothing.
        }
        state(uint8* ptr, uint32 offset, typename base::label_t s=DEFAULT) :
          base(ptr,offset,s) {
          // Do nothing.
        }
        state& operator=(typename base::label_t s) {
          base::operator=(s);
          return *this;
        }
        const std::string label() const {
          static const char* vec[]={
            "STANDBY_MODE",
            "MOTOR_TEST_MODE",
            "MOTOR_CTRL_MODE",
            "INERTIAL_CTRL_MODE_1",
            "INERTIAL_CTRL_MODE_2",
            "INERTIAL_CTRL_MODE_3",
            "TILT_COMP_MODE",
            "ALT_HOLD_MODE",
            "VEL_CTRL_MODE",
            "PVA_CTRL_MODE"
          };
          return vec[base::index()];
        }
        bool is_terminal() const {
          static const bool vec[]={
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false
          };
          return vec[base::index()];
        }
        bool can_transition_to(typename base::label_t s) const {
          bool ret = false;
          switch (base::index()) {
            case STANDBY_MODE:
              ret = (
                (s == STANDBY_MODE) or
                (s == MOTOR_TEST_MODE) or
                (s == MOTOR_CTRL_MODE) or
                (s == INERTIAL_CTRL_MODE_1) or
                (s == INERTIAL_CTRL_MODE_2) or
                (s == INERTIAL_CTRL_MODE_3) or
                (s == TILT_COMP_MODE) or
                (s == ALT_HOLD_MODE) or
                (s == VEL_CTRL_MODE) or
                (s == PVA_CTRL_MODE)
              );
              break;
            case MOTOR_TEST_MODE:
              ret = (
                (s == MOTOR_TEST_MODE) or
                (s == STANDBY_MODE)
              );
              break;
            case MOTOR_CTRL_MODE:
              ret = (
                (s == MOTOR_CTRL_MODE) or
                (s == STANDBY_MODE) or
                (s == INERTIAL_CTRL_MODE_1) or
                (s == INERTIAL_CTRL_MODE_2) or
                (s == INERTIAL_CTRL_MODE_3) or
                (s == TILT_COMP_MODE) or
                (s == ALT_HOLD_MODE) or
                (s == VEL_CTRL_MODE) or
                (s == PVA_CTRL_MODE)
              );
              break;
            case INERTIAL_CTRL_MODE_1:
              ret = (
                (s == INERTIAL_CTRL_MODE_1) or
                (s == STANDBY_MODE) or
                (s == MOTOR_CTRL_MODE) or
                (s == INERTIAL_CTRL_MODE_2) or
                (s == INERTIAL_CTRL_MODE_3) or
                (s == TILT_COMP_MODE) or
                (s == ALT_HOLD_MODE) or
                (s == VEL_CTRL_MODE) or
                (s == PVA_CTRL_MODE)
              );
              break;
            case INERTIAL_CTRL_MODE_2:
              ret = (
                (s == INERTIAL_CTRL_MODE_2) or
                (s == STANDBY_MODE) or
                (s == MOTOR_CTRL_MODE) or
                (s == INERTIAL_CTRL_MODE_1) or
                (s == INERTIAL_CTRL_MODE_3) or
                (s == TILT_COMP_MODE) or
                (s == ALT_HOLD_MODE) or
                (s == VEL_CTRL_MODE) or
                (s == PVA_CTRL_MODE)
              );
              break;
            case INERTIAL_CTRL_MODE_3:
              ret = (
                (s == INERTIAL_CTRL_MODE_3) or
                (s == STANDBY_MODE) or
                (s == MOTOR_CTRL_MODE) or
                (s == INERTIAL_CTRL_MODE_1) or
                (s == INERTIAL_CTRL_MODE_2) or
                (s == TILT_COMP_MODE) or
                (s == ALT_HOLD_MODE) or
                (s == VEL_CTRL_MODE) or
                (s == PVA_CTRL_MODE)
              );
              break;
            case TILT_COMP_MODE:
              ret = (
                (s == TILT_COMP_MODE) or
                (s == STANDBY_MODE) or
                (s == MOTOR_CTRL_MODE) or
                (s == INERTIAL_CTRL_MODE_1) or
                (s == INERTIAL_CTRL_MODE_2) or
                (s == INERTIAL_CTRL_MODE_3) or
                (s == ALT_HOLD_MODE) or
                (s == VEL_CTRL_MODE) or
                (s == PVA_CTRL_MODE)
              );
              break;
            case ALT_HOLD_MODE:
              ret = (
                (s == ALT_HOLD_MODE) or
                (s == STANDBY_MODE) or
                (s == MOTOR_CTRL_MODE) or
                (s == INERTIAL_CTRL_MODE_1) or
                (s == INERTIAL_CTRL_MODE_2) or
                (s == INERTIAL_CTRL_MODE_3) or
                (s == TILT_COMP_MODE) or
                (s == VEL_CTRL_MODE) or
                (s == PVA_CTRL_MODE)
              );
              break;
            case VEL_CTRL_MODE:
              ret = (
                (s == VEL_CTRL_MODE) or
                (s == STANDBY_MODE) or
                (s == MOTOR_CTRL_MODE) or
                (s == INERTIAL_CTRL_MODE_1) or
                (s == INERTIAL_CTRL_MODE_2) or
                (s == INERTIAL_CTRL_MODE_3) or
                (s == TILT_COMP_MODE) or
                (s == ALT_HOLD_MODE) or
                (s == PVA_CTRL_MODE)
              );
              break;
            case PVA_CTRL_MODE:
              ret = (
                (s == PVA_CTRL_MODE) or
                (s == STANDBY_MODE) or
                (s == MOTOR_CTRL_MODE) or
                (s == INERTIAL_CTRL_MODE_1) or
                (s == INERTIAL_CTRL_MODE_2) or
                (s == INERTIAL_CTRL_MODE_3) or
                (s == TILT_COMP_MODE) or
                (s == ALT_HOLD_MODE) or
                (s == VEL_CTRL_MODE)
              );
              break;
          }
          return ret;
        }
    };
  }

  #if defined(PRECOMPILE_COMM_LIB)
    class qctrl_mode_DEFAULT : public qctrl_mode::state<TOPIC_DEFAULT> {
      public:
        qctrl_mode_DEFAULT(typename base::label_t s=qctrl_mode::DEFAULT,typename base::node_id_t node_id=comm::meta::local_node_id());
        qctrl_mode_DEFAULT(const qctrl_mode_DEFAULT& rhs);
        qctrl_mode_DEFAULT(uint8* ptr, uint32 offset);
        ~qctrl_mode_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
