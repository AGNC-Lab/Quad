////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/producer.h                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_PRODUCER_H
#define UTILITIES_PRODUCER_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/consumer.h"
#include "utilities/globals.h"
#include "utilities/mutex.h"
#include "utilities/object.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  template <class T> class consumer;

  template <class T> class producer : public object {
    friend class consumer<T>;

    private:
      static mutex s_connection_mutex;

    private:
      std::string m_name;
      mutex m_consumer_list_mutex;
      std::list<consumer<T>*> m_consumer_list;

    public:
      producer(const std::string& name) : object("utilities","producer") {
        m_name = name;
      }
      ~producer() {
        // Lock "s_connection_mutex" to ensure that none of the consumers in "m_consumer_list" 
        // are destroyed "m_consumer_list" is being traversed.

        s_connection_mutex.lock();
          while (!m_consumer_list.empty()) {
            consumer<T>* p_consumer = m_consumer_list.front();
            base_disconnect_from(*p_consumer);
          }
        s_connection_mutex.unlock();
      }
      uint32 number_of_connections() const {
        uint32 ret;
        m_consumer_list_mutex.lock();
          ret = m_consumer_list.size();
        m_consumer_list_mutex.unlock();
        return ret;
      }
      bool is_connected() const {
        bool ret;
        m_consumer_list_mutex.lock();
          ret = !m_consumer_list.empty();
        m_consumer_list_mutex.unlock();
        return ret;
      }
      bool connect_to(consumer<T>& C) {
        // "s_connection_mutex" is used to avoid deadlocks between connecting/disconnecting 
        // producers and consumers. "m_consumer_list" and "consumer<T>::m_producer_list" are 
        // individually protected further by their respective mutexes in "base_connect_to()" 
        // and "base_disconnect_from()".

        s_connection_mutex.lock();
          bool performed_connection = base_connect_to(C);
        s_connection_mutex.unlock();
        return performed_connection;
      }
      bool disconnect_from(consumer<T>& C) {
        // "s_connection_mutex" is used to avoid deadlocks between connecting/disconnecting 
        // producers and consumers. "m_consumer_list" and "consumer<T>::m_producer_list" are 
        // individually protected further by their respective mutexes in "base_connect_to()" 
        // and "base_disconnect_from()".

        s_connection_mutex.lock();
          bool performed_disconnection = base_disconnect_from(C);
        s_connection_mutex.unlock();
        return performed_disconnection;
      }
      bool operator>>(consumer<T>& C) {
        return connect_to(C);
      }
      bool operator/=(consumer<T>& C) {
        return disconnect_from(C);
      }
      void push(const T& data) const {
        // "m_consumer_list_mutex" is used here to ensure that the push operation is atomic 
        // with respect to any connection/disconnection attempts. Data is pushed to all consumers 
        // in "m_consumer_list".
        typedef typename std::list<consumer<T>*>::const_iterator list_it;
        m_consumer_list_mutex.lock();
          for (list_it it=m_consumer_list.begin(); it!=m_consumer_list.end(); ++it) {
            (*it)->push(data);
          }
        m_consumer_list_mutex.unlock();
      }
    private:
      bool base_connect_to(consumer<T>& C) {
        // This function attempts to connect consumer "C" to the calling "producer<T>" object. 
        // This is done by getting the size of "m_consumer_list", and attempting to remove all 
        // instances of consumer "C" from it. If the size of the list is equal to the original 
        // size, then that means that "C" was not in the list to begin with, and therefore the 
        // function will proceed to connect "C". If the size is not equal to the original size, 
        // then this means that "C" was already connected, and a warning will be printed. 
        // Regardless, "C" is added back to "m_consumer_list" to ensure that it is in the list 
        // when the function returns.

        bool perfomed_connection;

        m_consumer_list_mutex.lock();
          C.m_producer_list_mutex.lock();
            uint32 original_size = m_consumer_list.size();
            m_consumer_list.remove(&C);
            perfomed_connection = (m_consumer_list.size() == original_size);
            m_consumer_list.push_back(&C);

            // Enter this branch if "C" is a new consumer.
            if (perfomed_connection) {
              // Add this producer to new consumer's producer list.
              C.m_producer_list.push_back(this);

              // Print out message upon successful connection.
              std::ostringstream msg;
              msg << m_name << " [" << m_consumer_list.size() << "] >> "
                  << C.m_name << " [" << C.m_producer_list.size() << "]";
              print(msg," - ");
            }
            // Enter this branch if "C" was already connected to this producer.
            else {
              warning("base_connect_to",m_name+" already connected to "+C.m_name);
            }
          C.m_producer_list_mutex.unlock();
        m_consumer_list_mutex.unlock();

        return perfomed_connection;
      }
      bool base_disconnect_from(consumer<T>& C) {
        bool performed_disconnection;

        m_consumer_list_mutex.lock();
          C.m_producer_list_mutex.lock();
            // This function attempts to disconnect consumer "C" from the calling "producer<T>" 
            // object. This is done by getting the size of "m_consumer_list", and attempting to 
            // remove all instances of consumer "C" from it. If the size of the list is equal to 
            // the original size, then that means that "C" was not in the list to begin with, 
            // and therefore the function will print out a warning. If the size is not equal to 
            // the original size, then this means that "C" was connected to the calling producer, 
            // and the function will proceed to disconnect "C".

            uint32 val = m_consumer_list.size();
            m_consumer_list.remove(&C);
            performed_disconnection = (m_consumer_list.size() != val);

            // Enter this branch if "C" was in "m_consumer_list".
            if (performed_disconnection) {
              // Remove this producer from consumer's producer list.
              C.m_producer_list.remove(this);

              // Print message upon successful disconnection.
              std::ostringstream msg;
              msg << m_name << " [" << m_consumer_list.size() << "] \\= "
                  << C.m_name << " [" << C.m_producer_list.size() << "]";
              print(msg," - ");
            }
            // Enter this branch if "C" was not connected to this producer.
            else {
              warning("base_disconnect_from",m_name+" was not connected to "+C.m_name);
            }
          C.m_producer_list_mutex.unlock();
        m_consumer_list_mutex.unlock();

        return performed_disconnection;
      }
  };

  template <class T> mutex producer<T>::s_connection_mutex;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
