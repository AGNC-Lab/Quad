////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/packet.h                                                                    //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_PACKET_H
#define UTILITIES_PACKET_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"
#include "utilities/serializable.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  template <class FAM,
            typename FAM::PACKET_ID_T PKT_ID,
            typename FAM::TOPIC_T TOPIC,
            uint32 N_PL_BYTES> class packet : public serializable {
    // FAM is a class with the following prototype:
    //  class FAM {
    //    public:
    //      enum PACKET_ID_T {};
    //      enum TOPIC_T {};
    //      enum NODE_ID_T {};
    //  
    //    public:
    //      static std::string packet_id_label(PACKET_ID_T i) {}
    //      static std::string topic_label(TOPIC_T i) {}
    //      static std::string node_id_label(NODE_ID_T i) {}
    //  };

    // List of pure virtual functions:
    // (1) std::ostream& payload_to_stream(std::ostream& stream) const

    public:
      typedef typename FAM::PACKET_ID_T pkt_id_t;
      typedef typename FAM::TOPIC_T topic_t;
      typedef typename FAM::NODE_ID_T node_id_t;
    private:
      typedef uint8 pkt_id_num_t;
      typedef uint8 topic_num_t;
      typedef uint8 node_id_num_t;

    // Common lengths and indices.
    protected:
      static const uint32 sc_i_pkt_id = 0;
      static const uint32 sc_i_topic = sc_i_pkt_id+sizeof(pkt_id_num_t);
      static const uint32 sc_i_node_id = sc_i_topic+sizeof(topic_num_t);
      static const uint32 sc_i_timestamp = sc_i_node_id+sizeof(node_id_num_t);
      static const uint32 sc_n_header_bytes = 7;
      static const uint32 sc_n_payload_bytes = N_PL_BYTES;
      static const uint32 sc_n_buffer_bytes = sc_n_header_bytes+sc_n_payload_bytes;
      static const uint32 sc_n_w_header_bytes = sc_n_header_bytes-sc_i_node_id;
      static const uint32 sc_n_w_bytes = sc_n_w_header_bytes+sc_n_payload_bytes;

    private:
      uint32 mc_n_header_bytes; // Number of header bytes owned by this instantiation.
      uint32 mc_n_buffer_bytes; // Number of buffer bytes owned this instantiation.
      uint8* const mcp_buffer; // Pointer to beginning of data buffer.
      uint8* const mcp_w_header; // Pointer to beginning of writable portion of header.
      uint8* const mcp_payload; // Pointer to beginning of payload.
      const bool mc_is_standalone; // True if packet is stand-alone (i.e. not a sub-packet).
      node_id_num_t& m_node_id; // Originating communication node ID.
      float& m_timestamp; // Packet timestamp [s].

    public:
      packet(node_id_t node_id) :
        mc_n_header_bytes(sc_n_header_bytes),
        mc_n_buffer_bytes(sc_n_buffer_bytes),
        mcp_buffer(static_cast<uint8*>(calloc(sc_n_buffer_bytes,1))),
        mcp_w_header(mcp_buffer+sc_i_node_id),
        mcp_payload(mcp_w_header+sc_n_w_header_bytes),
        mc_is_standalone(true),
        m_node_id(bind<node_id_num_t>(mcp_buffer,sc_i_node_id)),
        m_timestamp(bind<float>(mcp_buffer,sc_i_timestamp)) {
        // Use this constructor for stand-alone packets.
        set(mcp_buffer+sc_i_pkt_id,PKT_ID);
        set(mcp_buffer+sc_i_topic,TOPIC);
        set(mcp_buffer+sc_i_node_id,static_cast<node_id_num_t>(node_id));
        set(mcp_buffer+sc_i_timestamp,0.0f);
      }
      packet(const packet& rhs) :
        mc_n_header_bytes(sc_n_header_bytes),
        mc_n_buffer_bytes(sc_n_buffer_bytes),
        mcp_buffer(static_cast<uint8*>(malloc(sc_n_buffer_bytes))),
        mcp_w_header(mcp_buffer+sc_i_node_id),
        mcp_payload(mcp_w_header+sc_n_w_header_bytes),
        mc_is_standalone(true),
        m_node_id(bind<node_id_num_t>(mcp_buffer,sc_i_node_id)),
        m_timestamp(bind<float>(mcp_buffer,sc_i_timestamp)) {
        // This function is intended for use only between two stand-alone packets with identical 
        // packet IDs and topics. This assert is meant to avoid situations where the RHS has 
        // identical FAM, PKT_ID, T, and N_PL_BYTES arguments but is a sub-packet. This is 
        // made possible when topics are not unique to one packet ID.
        assert(rhs.mc_is_standalone);
        
        // Use this constructor for stand-alone packets.
        memcpy(mcp_w_header,rhs.mcp_w_header,sc_n_w_bytes);
      }
      packet(uint8* p_sup_pkt_pl, uint32 offset) :
        mc_n_header_bytes(0),
        mc_n_buffer_bytes(sc_n_payload_bytes),
        mcp_buffer(p_sup_pkt_pl+offset),
        mcp_w_header(NULL),
        mcp_payload(mcp_buffer),
        mc_is_standalone(false),
        m_node_id(bind<node_id_num_t>(p_sup_pkt_pl-sc_n_header_bytes,sc_i_node_id)),
        m_timestamp(bind<float>(p_sup_pkt_pl-sc_n_header_bytes,sc_i_timestamp)) {
        // Use this constructor for sub-packets.
        memset(mcp_payload,0,sc_n_payload_bytes);
      }
      virtual ~packet() {
        // Free allocated memory if memory was allocated by this instantiation.
        if (mc_is_standalone) {
          free(mcp_buffer);
        }
      }

      pkt_id_t packet_id_index() const {
        return PKT_ID;
      }
      topic_t topic_index() const {
        return TOPIC;
      }
      node_id_t node_id_index() const {
        return static_cast<node_id_t>(m_node_id);
      }

      std::string packet_id() const {
        return FAM::packet_id_label(PKT_ID);
      }
      std::string topic() const {
        return FAM::topic_label(TOPIC);
      }
      std::string node_id() const {
        return FAM::node_id_label(node_id_index());
      }

      float timestamp() const {
        return m_timestamp;
      }
      void timestamp(float ts) {
        m_timestamp = ts;
      }

      uint32 size() const {
        return mc_n_buffer_bytes;
      }
      uint32 header_size() const {
        return mc_n_header_bytes;
      }
      static uint32 payload_size() {
        return sc_n_payload_bytes;
      }

      uint8* serialize(uint8* buff) const {
        // This function is intended for use only with stand-alone packets.
        assert(mc_is_standalone);
        memcpy(buff,mcp_buffer,sc_n_buffer_bytes);
        return buff+sc_n_buffer_bytes;
      }
      uint8* serialize_header(uint8* buff) const {
        // This function is intended for use only with stand-alone packets.
        assert(mc_is_standalone);
        memcpy(buff,mcp_buffer,sc_n_header_bytes);
        return buff+sc_n_header_bytes;
      }
      uint8* serialize_payload(uint8* buff) const {
        memcpy(buff,mcp_payload,sc_n_payload_bytes);
        return buff+sc_n_payload_bytes;
      }

      const uint8* deserialize(const uint8* buff) {
        // This function is intended for use only with stand-alone packets.
        assert(mc_is_standalone);

        // Assumes that 'buff' points to the node ID element of the header (i.e. packet ID and 
        // topic are not copied).
        memcpy(mcp_w_header,buff,sc_n_w_bytes);
        return buff+sc_n_w_bytes;
      }
      const uint8* deserialize_header(const uint8* buff) {
        // This function is intended for use only with stand-alone packets.
        assert(mc_is_standalone);

        // Assumes that 'buff' points to the node ID element of the header (i.e. packet ID and 
        // topic are not copied).
        memcpy(mcp_w_header,buff,sc_n_w_header_bytes);
        return buff+sc_n_w_header_bytes;
      }
      const uint8* deserialize_payload(const uint8* buff) {
        memcpy(mcp_payload,buff,sc_n_payload_bytes);
        return buff+sc_n_payload_bytes;
      }

      packet& operator=(const packet& rhs) {
        // This function is intended for use only between two stand-alone packets with identical 
        // packet IDs and topics. This assert is meant to avoid situations where the RHS has 
        // identical FAM, PKT_ID, T, and N_PL_BYTES arguments but is a sub-packet. This is 
        // made possible when topics are not unique to one packet ID.
        assert(mc_is_standalone and rhs.mc_is_standalone);

        // Copies all writable bytes between packets with identical packet IDs and topics.
        memcpy(mcp_w_header,rhs.mcp_w_header,sc_n_w_bytes);

        return *this;
      }
      template <topic_t T> packet& operator<<=(const packet<FAM,PKT_ID,T,N_PL_BYTES>& rhs) {
        // This function is intended for use only with stand-alone packets.
        assert(mc_is_standalone);

        // Same as operator=(), but works between packets that have identical packet IDs (but 
        // possibly with different topics). Note that the RHS can be a sub-packet, but that the 
        // calling object (i.e. *this) must be a stand-alone packet.
        m_node_id = static_cast<node_id_num_t>(rhs.node_id_index());
        timestamp(rhs.timestamp());
        memcpy(mcp_payload,rhs.payload(),sc_n_payload_bytes);

        return *this;
      }
      template <topic_t T> void operator<<(const packet<FAM,PKT_ID,T,N_PL_BYTES>& rhs) {
        // Copies all payload bytes between packets with identical packet IDs (but possibly with 
        // different topics).
        memcpy(mcp_payload,rhs.payload(),sc_n_payload_bytes);
      }
      template <topic_t T> bool operator==(const packet<FAM,PKT_ID,T,N_PL_BYTES>& rhs) const {
        // Returns 'true' if payloads are identical between packets with identical packet IDs (but 
        // possibly with different topics).
        return (memcmp(mcp_payload,rhs.payload(),sc_n_payload_bytes) == 0);
      }
      template <topic_t T> bool operator!=(const packet<FAM,PKT_ID,T,N_PL_BYTES>& rhs) const {
        // Returns 'true' if payloads are not identical between packets with identical packet IDs 
        // (but possibly with different topics).
        return (memcmp(mcp_payload,rhs.payload(),sc_n_payload_bytes) != 0);
      }

      const uint8* payload() const {
        return mcp_payload;
      }
      virtual std::ostream& payload_to_stream(std::ostream& stream) const = 0;

    protected:
      uint8* payload() {
        return mcp_payload;
      }
      template <class T> T& bind(uint8* ptr, uint32 offset) {
        return *reinterpret_cast<T*>(ptr+offset);
      }
      template <class T> T& bind(uint32 offset) {
        return *reinterpret_cast<T*>(mcp_payload+offset);
      }
      template <class T> void set(uint8* ptr, T val) {
        *reinterpret_cast<T*>(ptr) = val;
      }

    private:
      std::ostream& base_payload_to_stream(std::ostream& stream) const {
        stream << std::setbase(10)
               << std::setfill('0') 
               << std::fixed
               << std::noshowpos
               << std::setprecision(0)
               << std::setw(3) << packet_id() << DELIMITER
               << std::setw(3) << topic() << DELIMITER
               << std::setw(3) << node_id() << DELIMITER
               << std::setprecision(3)
               << std::setw(9) << m_timestamp << DELIMITER;
        return payload_to_stream(stream);
      }
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
