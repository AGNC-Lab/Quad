////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/cv.cpp                                                                      //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/cv.h"
using utilities::cv;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  cv::cv() : object("utilities","cv") {
    int cond_var_errnum = pthread_cond_init(&m_cond_var,NULL);
    if (cond_var_errnum != 0) {
      error("cv","condition variable failed to initialize",cond_var_errnum);
    }

    int mutex_errnum = pthread_mutex_init(&m_mutex,NULL);
    if (mutex_errnum != 0) {
      error("cv","mutex failed to initialize",mutex_errnum);
    }

    m_wait_condition = true;
  }
  cv::~cv() {
    int cond_var_errnum = pthread_cond_destroy(&m_cond_var);
    if (cond_var_errnum != 0) {
      error("~cv","condition variable failed to destroy",cond_var_errnum);
    }

    int mutex_errnum = pthread_mutex_destroy(&m_mutex);
    if (mutex_errnum != 0) {
      error("~cv","mutex failed to destroy",mutex_errnum);
    }
  }
  void cv::wait() {
    pthread_mutex_lock(&m_mutex);
      while (m_wait_condition) {
        pthread_cond_wait(&m_cond_var,&m_mutex);
      }
      m_wait_condition = true;
    pthread_mutex_unlock(&m_mutex);
  }
  void cv::signal() {
    pthread_mutex_lock(&m_mutex);
      m_wait_condition = false;
      pthread_cond_signal(&m_cond_var);
    pthread_mutex_unlock(&m_mutex);
  }
  void cv::broadcast() {
    pthread_mutex_lock(&m_mutex);  
      m_wait_condition = false;
      pthread_cond_broadcast(&m_cond_var);
    pthread_mutex_unlock(&m_mutex);
  }
#elif (QT_ENV)
  cv::cv() : object("utilities","cv") {
    m_wait_condition = true;
  }
  cv::~cv() {

  }
  void cv::wait() {
    m_mutex.lock();
      while (m_wait_condition) {
        m_cond_var.wait(&m_mutex);
      }
      m_wait_condition = true;
    m_mutex.unlock();
  }
  void cv::signal() {
    m_mutex.lock();
      m_wait_condition = false;
      m_cond_var.wakeOne();
    m_mutex.unlock();
  }
  void cv::broadcast() {
    m_mutex.lock();
      m_wait_condition = false;
      m_cond_var.wakeAll();
    m_mutex.unlock();
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
