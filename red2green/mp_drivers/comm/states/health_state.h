////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   states/health_state.h                                                                 //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STATES_HEALTH_STATE_H
#define STATES_HEALTH_STATE_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/state.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace states {
  namespace health_state {
    enum label_t {
      GREEN,
      YELLOW,
      RED,
      BEGIN=GREEN,
      END=RED,
      DEFAULT=GREEN
    };

    template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
    class state : public utilities::state<comm::meta,comm::meta::ID_HEALTH_STATE,TOPIC,label_t> {
      public:
        typedef utilities::state<comm::meta,comm::meta::ID_HEALTH_STATE,TOPIC,label_t> base;

      public:
        state(typename base::label_t s=DEFAULT,
              typename base::node_id_t n=comm::meta::local_node_id()) :
          base(s,n) {
          assert(
            (TOPIC == comm::meta::HMC5883L) or
            (TOPIC == comm::meta::MPU6050) or
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
            "GREEN",
            "YELLOW",
            "RED"
          };
          return vec[base::index()];
        }
        bool is_terminal() const {
          static const bool vec[]={
            false,
            false,
            false
          };
          return vec[base::index()];
        }
        bool can_transition_to(typename base::label_t s) const {
          bool ret = false;
          switch (base::index()) {
            case GREEN:
              ret = (
                (s == GREEN) or
                (s == RED)
              );
              break;
            case YELLOW:
              ret = (
                (s == YELLOW) or
                (s == RED)
              );
              break;
            case RED:
              ret = (
                (s == RED) or
                (s == YELLOW)
              );
              break;
          }
          return ret;
        }
    };
  }

  #if defined(PRECOMPILE_COMM_LIB)
    class health_state_HMC5883L : public health_state::state<TOPIC_HMC5883L> {
      public:
        health_state_HMC5883L(typename base::label_t s=health_state::DEFAULT,typename base::node_id_t node_id=comm::meta::local_node_id());
        health_state_HMC5883L(const health_state_HMC5883L& rhs);
        health_state_HMC5883L(uint8* ptr, uint32 offset);
        ~health_state_HMC5883L();
    };
    class health_state_MPU6050 : public health_state::state<TOPIC_MPU6050> {
      public:
        health_state_MPU6050(typename base::label_t s=health_state::DEFAULT,typename base::node_id_t node_id=comm::meta::local_node_id());
        health_state_MPU6050(const health_state_MPU6050& rhs);
        health_state_MPU6050(uint8* ptr, uint32 offset);
        ~health_state_MPU6050();
    };
    class health_state_DEFAULT : public health_state::state<TOPIC_DEFAULT> {
      public:
        health_state_DEFAULT(typename base::label_t s=health_state::DEFAULT,typename base::node_id_t node_id=comm::meta::local_node_id());
        health_state_DEFAULT(const health_state_DEFAULT& rhs);
        health_state_DEFAULT(uint8* ptr, uint32 offset);
        ~health_state_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
