////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   states/mag_state.h                                                                    //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STATES_MAG_STATE_H
#define STATES_MAG_STATE_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/state.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace states {
  namespace mag_state {
    enum label_t {
      IDLE,
      DEVICE,
      MOCAP,
      SIM,
      CALIBRATE,
      TERM,
      BEGIN=IDLE,
      END=TERM,
      DEFAULT=IDLE
    };

    template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
    class state : public utilities::state<comm::meta,comm::meta::ID_MAG_STATE,TOPIC,label_t> {
      public:
        typedef utilities::state<comm::meta,comm::meta::ID_MAG_STATE,TOPIC,label_t> base;

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
            "IDLE",
            "DEVICE",
            "MOCAP",
            "SIM",
            "CALIBRATE",
            "TERM"
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
            true
          };
          return vec[base::index()];
        }
        bool can_transition_to(typename base::label_t s) const {
          bool ret = false;
          switch (base::index()) {
            case IDLE:
              ret = (
                (s == IDLE) or
                (s == TERM) or
                (s == DEVICE) or
                (s == MOCAP) or
                (s == SIM)
              );
              break;
            case DEVICE:
              ret = (
                (s == DEVICE) or
                (s == IDLE) or
                (s == MOCAP) or
                (s == CALIBRATE)
              );
              break;
            case MOCAP:
              ret = (
                (s == MOCAP) or
                (s == IDLE) or
                (s == DEVICE)
              );
              break;
            case SIM:
              ret = (
                (s == SIM) or
                (s == IDLE)
              );
              break;
            case CALIBRATE:
              ret = (
                (s == CALIBRATE) or
                (s == DEVICE)
              );
              break;
            case TERM:
              ret = (
                (s == TERM) or
                (s == IDLE)
              );
              break;
          }
          return ret;
        }
    };
  }

  #if defined(PRECOMPILE_COMM_LIB)
    class mag_state_DEFAULT : public mag_state::state<TOPIC_DEFAULT> {
      public:
        mag_state_DEFAULT(typename base::label_t s=mag_state::DEFAULT,typename base::node_id_t node_id=comm::meta::local_node_id());
        mag_state_DEFAULT(const mag_state_DEFAULT& rhs);
        mag_state_DEFAULT(uint8* ptr, uint32 offset);
        ~mag_state_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
