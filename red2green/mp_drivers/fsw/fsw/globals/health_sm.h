////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/globals/health_sm.h                                                               //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FSW_GLOBAL_HEALTH_SM_H
#define FSW_GLOBAL_HEALTH_SM_H

#include "fsw/globals/globals.h"
#include "states/health_state.h"
#include "utilities/consumer.h"
#include "utilities/producer.h"
#include "utilities/state_machine.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace fsw {
	namespace globals {
    template <comm::meta::TOPIC_T T>
    class health_sm :
      protected utilities::state_machine< typename states::health_state::state<T> > {
      public:
        typedef utilities::state_machine< typename states::health_state::state<T> > base;
        
      public:
        utilities::consumer<bool> rx_reset;
        utilities::producer<typename base::state>& tx_current_state;

      public:
        health_sm(const std::string& name) : 
          base(name+"::health_sm",states::health_state::GREEN,comm::meta::local_node_id()),
          rx_reset(name+"::rx_reset",100),
          tx_current_state(base::tx_current_state) {
          // Do nothing.
        }
        void report_error() {
          base::rx_transition_request.push(states::health_state::RED);
        }
        void clear_error() {
          base::force_to(states::health_state::GREEN);
        }
        void broadcast_current_state(float ts) {
          // Clear error if rx_reset was signaled.
          if (rx_reset.pull()) {
            clear_error();
          }
          
          // Transition back to YELLOW if no new valid state transition request is received and 
          // if current state is RED. Since rx_transition_request consumer is protected, it can 
          // only have RED pushed into it through report_error(). Therefore, any time 
          // process_transition_request() returns true, it is guaranteed that the current state 
          // is RED. When GREEN, the state machine should never transition to YELLOW, and if a 
          // new error was reported, the state machine should maintain RED state.
          if ((!base::process_transition_requests()) and
              (base::current_state() == states::health_state::RED)) {
            base::transition_to(states::health_state::YELLOW);
          }
          
          // Flush rx_st_req consumer.
          base::rx_transition_request.flush();
          
          // Broadcast current state.
          base::broadcast_current_state(ts);
        }
    };
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
