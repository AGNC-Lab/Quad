////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/params.h                                                                    //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_PARAMS_H
#define UTILITIES_PARAMS_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"
#include "utilities/packet.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  template <class FAM,
            typename FAM::PACKET_ID_T PKT_ID,
            typename FAM::TOPIC_T TOPIC,
            uint32 N_PL_BYTES> class params : public packet<FAM,PKT_ID,TOPIC,N_PL_BYTES> {
    // List of pure virtual functions:
    // (1) std::ostream& payload_to_stream(std::ostream& stream) const
    // (2) void load(const std::string& prefix, ifile& file)
    // (3) void save(const std::string& prefix, const ofile& file) const

    public:
      typedef packet<FAM,PKT_ID,TOPIC,N_PL_BYTES> base;
      typedef typename FAM::TOPIC_T topic_t;
      typedef typename FAM::NODE_ID_T node_id_t;

    public:
      params(node_id_t node_id) : base(node_id) {
        // Do nothing.
      }
      params(const params& rhs) : base(rhs) {
        // Do nothing.
      }
      params(uint8* p_sup_pkt_pl, uint32 offset) : base(p_sup_pkt_pl,offset) {
        // Do nothing.
      }
      virtual ~params() {
        // Do nothing.
      }

      virtual void load_elems(const std::string& prefix, const ifile& file) = 0;
      virtual void save_elems(const std::string& prefix, ofile& file) const = 0;
      virtual void fill() {
        // No-op.
      }

      void load(const std::string& filename) {
        message(FAM::packet_id_label(PKT_ID)+"<"+FAM::topic_label(TOPIC)+">::load",
                "loading files from "+filename);

        ifile file(filename);
        load_elems("",file);
        fill();

        message(FAM::packet_id_label(PKT_ID)+"<"+FAM::topic_label(TOPIC)+">::load",
                "loading complete");
      }
      void save(const std::string& filename) const {
        message(FAM::packet_id_label(PKT_ID)+"<"+FAM::topic_label(TOPIC)+">::save",
                "saving files to "+filename);

        ofile file(filename,ofile::OVERWRITE);
        save_elems("",file);

        message(FAM::packet_id_label(PKT_ID)+"<"+FAM::topic_label(TOPIC)+">::save",
                "saving complete");
      }
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
