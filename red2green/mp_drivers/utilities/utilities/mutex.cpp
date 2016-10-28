////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/mutex.cpp                                                                   //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/mutex.h"
using utilities::mutex;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  mutex::mutex() : object("utilities","mutex") {
    int errnum = pthread_mutex_init(&m_mutex,NULL);
    if (errnum != 0) {
      error("mutex","failed to initialize",errnum);
    }
    mp_mutex = &m_mutex;
  }
  mutex::~mutex() {
    int errnum = pthread_mutex_destroy(&m_mutex);
    if (errnum != 0) {
      error("~mutex","failed to destroy",errnum);
    }
  }
  void mutex::lock() const {
    pthread_mutex_lock(mp_mutex);
  }
  bool mutex::try_lock() const {
    int ret = pthread_mutex_trylock(mp_mutex);
    return (ret == 0);
  }
  void mutex::unlock() const {
    pthread_mutex_unlock(mp_mutex);
  }
#elif (QT_ENV)
  mutex::mutex() : object("utilities","mutex") {
    mp_mutex = &m_mutex;
  }
  mutex::~mutex() {
    // Do nothing.
  }
  void mutex::lock() const {
    mp_mutex->lock();
  }
  bool mutex::try_lock() const {
    return mp_mutex->tryLock();
  }
  void mutex::unlock() const {
    mp_mutex->unlock();
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
