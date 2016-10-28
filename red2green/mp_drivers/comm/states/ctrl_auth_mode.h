////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   states/ctrl_auth_mode.h                                                               //
// AUTHORS: Miki Szmuk                                                                            //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STATES_CTRL_AUTH_MODE_H
#define STATES_CTRL_AUTH_MODE_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "comm/globals.h"
#include "utilities/globals.h"
#include "utilities/state.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace states {
  namespace ctrl_auth_mode {
    enum label_t {
      GCS_CTRL,
      API_CTRL,
      BEGIN=GCS_CTRL,
      END=API_CTRL,
      DEFAULT=GCS_CTRL
    };

    template <comm::meta::TOPIC_T TOPIC=comm::meta::DEFAULT>
    class state : public utilities::state<comm::meta,comm::meta::ID_CTRL_AUTH_MODE,TOPIC,label_t> {
      public:
        typedef utilities::state<comm::meta,comm::meta::ID_CTRL_AUTH_MODE,TOPIC,label_t> base;

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
            "GCS_CTRL",
            "API_CTRL"
          };
          return vec[base::index()];
        }
        bool is_terminal() const {
          static const bool vec[]={
            false,
            false
          };
          return vec[base::index()];
        }
        bool can_transition_to(typename base::label_t s) const {
          bool ret = false;
          switch (base::index()) {
            case GCS_CTRL:
              ret = (
                (s == GCS_CTRL) or
                (s == API_CTRL)
              );
              break;
            case API_CTRL:
              ret = (
                (s == API_CTRL) or
                (s == GCS_CTRL)
              );
              break;
          }
          return ret;
        }
    };
  }

  #if defined(PRECOMPILE_COMM_LIB)
    class ctrl_auth_mode_DEFAULT : public ctrl_auth_mode::state<TOPIC_DEFAULT> {
      public:
        ctrl_auth_mode_DEFAULT(typename base::label_t s=ctrl_auth_mode::DEFAULT,typename base::node_id_t node_id=comm::meta::local_node_id());
        ctrl_auth_mode_DEFAULT(const ctrl_auth_mode_DEFAULT& rhs);
        ctrl_auth_mode_DEFAULT(uint8* ptr, uint32 offset);
        ~ctrl_auth_mode_DEFAULT();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
