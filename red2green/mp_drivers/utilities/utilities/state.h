////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/state.h                                                                     //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_STATE_H
#define UTILITIES_STATE_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"
#include "utilities/packet.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  template <class FAM,
            typename FAM::PACKET_ID_T PKT_ID,
            typename FAM::TOPIC_T TOPIC,
            class LABEL_T> class state : public packet<FAM,PKT_ID,TOPIC,1> {
    // List of pure virtual functions:
    // (1) const std::string label() const
    // (2) bool is_terminal() const
    // (3) bool can_transition_to(label_t s) const

    public:
      typedef packet<FAM,PKT_ID,TOPIC,1> base;
      typedef typename FAM::TOPIC_T topic_t;
      typedef typename FAM::NODE_ID_T node_id_t;
      typedef LABEL_T label_t;

    private:
      uint8& m_state_index;

    public:
      state(label_t s, node_id_t node_id) :
        base(node_id),
        m_state_index(base::template bind<uint8>(0)) {
        m_state_index = s;
      }
      state(const state& rhs) :
        base(rhs),
        m_state_index(base::template bind<uint8>(0)) {
        // Do nothing.
      }
      state(uint8* ptr, uint32 offset, label_t s) :
        base(ptr,offset),
        m_state_index(base::template bind<uint8>(0)) {
        m_state_index = s;
      }
      virtual ~state() {
        // Do nothing.
      }

      state& operator=(const state& rhs) {
        base::operator=(rhs);
        return *this;
      }
      template <topic_t T> bool operator==(const state<FAM,PKT_ID,T,LABEL_T>& s) const {
        return (m_state_index == s.index());
      }
      template <topic_t T> bool operator!=(const state<FAM,PKT_ID,T,LABEL_T>& s) const {
        return (m_state_index != s.index());
      }
      template <topic_t T> bool operator<(const state<FAM,PKT_ID,T,LABEL_T>& s) const {
        return (m_state_index < s.index());
      }
      template <topic_t T> bool operator>(const state<FAM,PKT_ID,T,LABEL_T>& s) const {
        return (m_state_index > s.index());
      }
      template <topic_t T> bool operator<=(const state<FAM,PKT_ID,T,LABEL_T>& s) const {
        return (m_state_index <= s.index());
      }
      template <topic_t T> bool operator>=(const state<FAM,PKT_ID,T,LABEL_T>& s) const {
        return (m_state_index >= s.index());
      }

      state& operator=(label_t s) {
        m_state_index = s;
        return *this;
      }
      bool operator==(label_t s) const {
        return (m_state_index == s);
      }
      bool operator!=(label_t s) const {
        return (m_state_index != s);
      }
      bool operator<(label_t s) const {
        return (m_state_index < s);
      }
      bool operator>(label_t s) const {
        return (m_state_index > s);
      }
      bool operator<=(label_t s) const {
        return (m_state_index <= s);
      }
      bool operator>=(label_t s) const {
        return (m_state_index >= s);
      }

      label_t index() const {
        return static_cast<label_t>(m_state_index);
      }
      std::ostream& payload_to_stream(std::ostream& stream) const {
        stream << label();
        return stream;
      }

      virtual const std::string label() const = 0;
      virtual bool is_terminal() const = 0;
      virtual bool can_transition_to(label_t s) const = 0;

    friend bool operator==(label_t s_lhs, const state& s_rhs) {
      return (s_lhs == s_rhs.m_state_index);
    }
    friend bool operator!=(label_t s_lhs, const state& s_rhs) {
      return (s_lhs != s_rhs.m_state_index);
    }
    friend bool operator<(label_t s_lhs, const state& s_rhs) {
      return (s_lhs < s_rhs.m_state_index);
    }
    friend bool operator>(label_t s_lhs, const state& s_rhs) {
      return (s_lhs > s_rhs.m_state_index);
    }
    friend bool operator<=(label_t s_lhs, const state& s_rhs) {
      return (s_lhs <= s_rhs.m_state_index);
    }
    friend bool operator>=(label_t s_lhs, const state& s_rhs) {
      return (s_lhs >= s_rhs.m_state_index);
    }
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
