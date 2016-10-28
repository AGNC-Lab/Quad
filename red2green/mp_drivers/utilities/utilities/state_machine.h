////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/state_machine.h                                                             //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_STATE_MACHINE_H
#define UTILITIES_STATE_MACHINE_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/consumer.h"
#include "utilities/globals.h"
#include "utilities/mutex.h"
#include "utilities/producer.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  template <class STATE> class state_machine {
    public:
      typedef STATE state;

    public:
      consumer<state> rx_transition_request;
      producer<state> tx_current_state;

    private:
      const std::string mc_name;
      state m_previous_state;
      state m_current_state;
      mutex m_mutex;

    public:
      state_machine(const std::string& name,
                    typename state::label_t state_i) :
        rx_transition_request(name+"::rx_transition_request",100),
        tx_current_state(name+"::tx_current_state"),
        mc_name(name),
        m_previous_state(state_i),
        m_current_state(state_i) {
        // Do nothing.
      }
      state_machine(const std::string& name,
                    typename state::label_t state_i,
                    typename state::node_id_t node_id) :
        rx_transition_request(name+"::rx_transition_request",100),
        tx_current_state(name+"::tx_current_state"),
        mc_name(name),
        m_previous_state(state_i,node_id),
        m_current_state(state_i,node_id) {
        // Do nothing.
      }
      virtual ~state_machine() {
        // Do nothing.
      }
      state previous_state() const {
        state ret;
        m_mutex.lock();
          ret = m_previous_state;
        m_mutex.unlock();
        return ret;
      }
      state current_state() const {
        state ret;
        m_mutex.lock();
          ret = m_current_state;
        m_mutex.unlock();
        return ret;
      }
      bool transition_to(const state& next_state) {
        // This function attempts to transition the state from "m_current_state" to "next_state". 
        // The transition is allowed only if "m_current_state" is not terminal, and if 
        // the "can_transition_to()" function returns "true". This function returns "true" if the 
        // transition is allowed, and "false" otherwise.

        bool ret;
        m_mutex.lock();
          if (m_current_state.is_terminal()) {
            #if !defined(SUPPRESS_STATE_MACHINE_MESSAGES)
              std::ostringstream msg;
              msg << mc_name << " transition: "
                << "|" << m_current_state.label() << "|"
                << " -/-> "
                << next_state.label();
              print(msg," * ");
            #endif
            ret = false;
          }
          else if (m_current_state.can_transition_to(next_state.index())) {
            m_previous_state << m_current_state;
            m_current_state << next_state;
            #if !defined(SUPPRESS_STATE_MACHINE_MESSAGES)
              std::ostringstream msg;
              msg << mc_name << " transition: "
                  << m_previous_state.label()
                  << " ---> "
                  << m_current_state.label();
              print(msg," > ");
            #endif
            ret = true;
          }
          else {
            #if !defined(SUPPRESS_STATE_MACHINE_MESSAGES)
              std::ostringstream msg;
              msg << mc_name << " transition: "
                  << m_current_state.label()
                  << " -/-> "
                  << next_state.label();
              print(msg," * ");
            #endif
            ret = false;
          }
        m_mutex.unlock();
        return ret;
      }
      bool push_to(const state& next_state) {
        // This function attempts to transition the state from "m_current_state" to "next_state". 
        // The transition is allowed only if the "can_transition_to()" function 
        // returns "true". In this function, the transition does not depend on whether 
        // "m_current_state" is terminal. This function returns "true" if the state transition 
        // is allowed, and "false" otherwise.

        bool ret;
        m_mutex.lock();
          if (m_current_state.can_transition_to(next_state.index())) {
            m_previous_state << m_current_state;
            m_current_state << next_state;
            #if !defined(SUPPRESS_STATE_MACHINE_MESSAGES)
              std::ostringstream msg;
              msg << mc_name << " push: "
                  << m_previous_state.label()
                  << " ---> "
                  << m_current_state.label();
              print(msg," > ");
            #endif
            ret = true;
          }
          else {
            #if !defined(SUPPRESS_STATE_MACHINE_MESSAGES)
              std::ostringstream msg;
              msg << mc_name << " push: "
                  << m_current_state.label()
                  << " -/-> "
                  << next_state.label();
              print(msg," * ");
            #endif
            ret = false;
          }
        m_mutex.unlock();
        return ret;
      }
      void force_to(const state& next_state) {
        // This function transitions the state from "m_current_state" to "next_state", irrespective 
        // of the output of "can_transition_to()" or if "m_current_state" is a terminal state.

        m_mutex.lock();
          m_previous_state << m_current_state;
          m_current_state << next_state;
          #if !defined(SUPPRESS_STATE_MACHINE_MESSAGES)
            std::ostringstream msg;
            msg << mc_name << " force: "
                << m_previous_state.label()
                << " ---> "
                << m_current_state.label();
            print(msg," * ");
          #endif
        m_mutex.unlock();
      }
      bool transition_to(typename state::label_t next_state) {
        return transition_to(state(next_state));
      }
      bool push_to(typename state::label_t next_state) {
        return push_to(state(next_state));
      }
      void force_to(typename state::label_t next_state) {
        force_to(state(next_state));
      }
      bool process_transition_requests() {
        state next_state;
        if (rx_transition_request.pull(next_state)) {
          return transition_to(next_state);
        }
        else {
          return false;
        }
      }
      virtual void broadcast_current_state(float ts) const {
        state current_state = m_current_state;
        current_state.timestamp(ts);
        tx_current_state.push(current_state);
      }
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
