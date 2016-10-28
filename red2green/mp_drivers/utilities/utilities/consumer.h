////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/consumer.h                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_CONSUMER_H
#define UTILITIES_CONSUMER_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"
#include "utilities/mutex.h"
#include "utilities/producer.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  template <class T> class producer;

  template <class T> class consumer {
    friend class producer<T>;

    private:
      std::string m_name;
      const uint32 mc_max_fifo_size;
      mutex m_producer_list_mutex;
      std::list<producer<T>*> m_producer_list;
      mutex m_data_fifo_mutex;
      std::queue<T> m_data_fifo;

    public:
      consumer(const std::string& name, uint32 max_fifo_size) : mc_max_fifo_size(max_fifo_size) {
        m_name = name;
      }
      ~consumer() {
        // Lock "producer<T>::s_connection_mutex" to ensure that none of the producers in 
        // "m_producer_list" are destroyed while "m_producer_list" is being traversed.

        producer<T>::s_connection_mutex.lock();
          while (!m_producer_list.empty()) {
            producer<T>* p_producer = m_producer_list.front();
            p_producer->base_disconnect_from(*this);
          }
        producer<T>::s_connection_mutex.unlock();
      }
      uint32 number_of_connections() const {
        uint32 ret;
        m_producer_list_mutex.lock();
          ret = m_producer_list.size();
        m_producer_list_mutex.unlock();
        return ret;
      }
      bool is_connected() const {
        bool ret;
        m_producer_list_mutex.lock();
          ret = !m_producer_list.empty();
        m_producer_list_mutex.unlock();
        return ret;
      }
      bool connect_to(producer<T>& P) {
        return P.connect_to(*this);
      }
      bool disconnect_from(producer<T>& P) {
        return P.disconnect_from(*this);
      }
      bool operator<<(producer<T>& P) {
        return P.connect_to(*this);
      }
      bool operator/=(producer<T>& P) {
        return P.disconnect_from(*this);
      }
      bool new_data_available() const {
        bool ret;
        m_data_fifo_mutex.lock();
          ret = !m_data_fifo.empty();
        m_data_fifo_mutex.unlock();
        return ret;
      }
      uint32 queue_length() const {
        uint32 ret;
        m_data_fifo_mutex.lock();
          ret = m_data_fifo.size();
        m_data_fifo_mutex.unlock();
        return ret;
      }
      void flush() {
        // Empties "m_data_fifo".

        m_data_fifo_mutex.lock();
          m_data_fifo = std::queue<T>();
        m_data_fifo_mutex.unlock();
      }
      void push(const T& data) {
        // Copies "data" into "m_data_fifo", thus making it the newest element of the queue. If 
        // the length of "m_data_fifo" is equal to "mc_max_fifo_size", remove the oldest 
        // element in the queue. This function is public to allow for producer-less / 
        // connection-less pushes.
        m_data_fifo_mutex.lock();
          if (m_data_fifo.size() == mc_max_fifo_size) {
            m_data_fifo.pop();
          }
          m_data_fifo.push(data);
        m_data_fifo_mutex.unlock();
      }
      bool peek(T& data) const {
        // Copies the oldest data in "m_data_fifo" to "data", but does not remove it from 
        // "m_data_fifo". Returns "true" if "m_data_fifo" was not empty.

        bool peeked_at_new_data = false;
        m_data_fifo_mutex.lock();
          if (!m_data_fifo.empty()) {
            peeked_at_new_data = true;
            data = m_data_fifo.front();
          }
        m_data_fifo_mutex.unlock();
        return peeked_at_new_data;
      }
      bool pull(T& data) {
        // Removes the oldest data in "m_data_fifo" and copies it to "data". Returns "true" if 
        // "m_data_fifo" was not empty.

        bool pulled_new_data = false;
        m_data_fifo_mutex.lock();
          if (!m_data_fifo.empty()) {
            pulled_new_data = true;
            data = m_data_fifo.front();
            m_data_fifo.pop();
          }
        m_data_fifo_mutex.unlock();
        return pulled_new_data;
      }
      bool peek() {
        // Copies the oldest data in "m_data_fifo" to "data", but does not remove it from 
        // "m_data_fifo". Returns "true" if "m_data_fifo" was not empty.

        bool peeked_at_new_data = false;
        m_data_fifo_mutex.lock();
          if (!m_data_fifo.empty()) {
            peeked_at_new_data = true;
          }
        m_data_fifo_mutex.unlock();
        return peeked_at_new_data;
      }
      bool pull() {
        // Removes the oldest data in "m_data_fifo" and copies it to "data". Returns "true" if 
        // "m_data_fifo" was not empty.

        bool pulled_new_data = false;
        m_data_fifo_mutex.lock();
          if (!m_data_fifo.empty()) {
            pulled_new_data = true;
            m_data_fifo.pop();
          }
        m_data_fifo_mutex.unlock();
        return pulled_new_data;
      }
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
