////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/thread.cpp                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/thread.h"
  #include <sys/resource.h>
#elif (QT_ENV)
  #include "utilities/thread.h"
#endif
using std::ostringstream;
using std::string;
using utilities::mutex;
using utilities::thread;

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  uint32 thread::s_num_threads_running = 0;
  mutex thread::s_num_threads_running_mutex;

  thread::thread(const string& name, PRIORITY priority) :
    object("utilities","thread"),
    mc_name(name),
    mc_priority(priority) {
    m_is_running = false;
    m_handle = 0;
    m_keep_running = false;
  }
  thread::~thread() {
    if (is_running()) {
      warning("~thread",mc_name+" was not terminated prior to destruction");
    }
  }
  const string& thread::name() const {
    return mc_name;
  }
  bool thread::is_running() const {
    m_is_running_mutex.lock();
      bool ret = m_is_running;
    m_is_running_mutex.unlock();
    return ret;
  }
  bool thread::launch() {
    if (is_running()) {
      warning("launch",mc_name+" is already running");
      return false;
    }
    else {
      // Lower thread termination flag.
      m_keep_running_mutex.lock();
        m_keep_running = true;
      m_keep_running_mutex.unlock();

      // Attempt to create thread.
      int32 errnum = pthread_create(&m_handle,NULL,thread_function,(void*)this);
      if (errnum != 0) {
        error("launch","failed to launch "+mc_name,errnum);
        return false;
      }
      else {
        s_num_threads_running_mutex.lock();
          ++s_num_threads_running;

          ostringstream msg;
          msg << "launched " << mc_name << " [" << s_num_threads_running << "]";
          message("launch",msg.str());
        s_num_threads_running_mutex.unlock();

        m_is_running_mutex.lock();
          m_is_running = true;
        m_is_running_mutex.unlock();
        return true;
      }
    }
  }
  bool thread::terminate() {
    if (is_running()) {
      message("terminate","sending termination signal to "+mc_name);

      // Flag thread for termination.
      m_keep_running_mutex.lock();
        m_keep_running = false;
      m_keep_running_mutex.unlock();

      // Attempt to join thread (i.e. wait until thread_function() returns).
      void* ret = NULL;
      int32 errnum = pthread_join(m_handle,&ret);

      // Once joined, set m_is_running to false.
      m_is_running_mutex.lock();
        m_is_running = false;
      m_is_running_mutex.unlock();

      if (errnum != 0) {
        error("terminate",mc_name+" failed to join",errnum);
        return false;
      }
      else {
        ostringstream msg;
        msg << mc_name;

        if (ret == PTHREAD_CANCELED) {
          msg << " was cancelled";
        }
        else {
          msg << " terminated normally";
        }

        s_num_threads_running_mutex.lock();
          --s_num_threads_running;
          msg << " [" << s_num_threads_running << "]";
          message("terminate",msg.str());
        s_num_threads_running_mutex.unlock();
        return true;
      }
    }
    else {
      return false;
    }
  }
  bool thread::keep_running() {
    m_keep_running_mutex.lock();
      bool ret = m_keep_running;
    m_keep_running_mutex.unlock();
    return ret;
  }
  void thread::execute() {
    // Implement custom thread loop function in a derived class.

    // IMPORTANT: Care must be taken with classes derived from this object. In particular, make 
    // sure to call the terminate() function in the derived class's destructor so that the 
    // execute() function returns prior to the destruction of derived members. If execute() is not 
    // terminated prior to the destruction of derived members, execute() may attempt to access an 
    // object that no longer exists.
  }
  void* thread::thread_function(void* arg) {
    thread* ptr = (thread*)arg;
    setpriority(PRIO_PROCESS,0,ptr->mc_priority);
    ptr->execute();
    pthread_exit(NULL);
    return 0;
  }
#elif (QT_ENV)
  uint32 thread::s_num_threads_running = 0;
  mutex thread::s_num_threads_running_mutex;

  thread::thread(const std::string& name, PRIORITY priority) :
    object("utilities","thread"),
    mc_name(name),
    mc_priority(priority) {
    m_keep_running = false;

    // Disable QThread termination.
    QThread::setTerminationEnabled(false);
  }
  thread::~thread() {
    if (is_running()) {
      warning("~thread",mc_name+" was not terminated prior to destruction");
    }
  }
  const string& thread::name() const {
    return mc_name;
  }
  bool thread::is_running() const {
    return QThread::isRunning();
  }
  bool thread::launch() {
    if (is_running()) {
      warning("launch",mc_name+" is already running");
      return false;
    }
    else {
      // Lower thread termination flag.
      m_keep_running_mutex.lock();
        m_keep_running = true;
      m_keep_running_mutex.unlock();

      // Attempt to create thread.
      QThread::start();

      if (!is_running()) {
        warning("launch","failed to launch "+mc_name);
        return false;
      }
      else {
        // Set thread priority.
        QThread::setPriority((QThread::Priority)mc_priority);

        s_num_threads_running_mutex.lock();
          ++s_num_threads_running;

          ostringstream msg;
          msg << "launched " << mc_name << " [" << s_num_threads_running << "]";
          message("launch",msg.str());
        s_num_threads_running_mutex.unlock();

        return true;
      }
    }
  }
  bool thread::terminate() {
    if (is_running()) {
      message("terminate","sending termination signal to "+mc_name);

      // Flag thread for termination.
      m_keep_running_mutex.lock();
        m_keep_running = false;
      m_keep_running_mutex.unlock();

      // Attempt to join thread (i.e. wait until thread_function() returns).
      QThread::wait();

      if (is_running()) {
        warning("terminate",mc_name+" failed to join");
        return false;
      }
      else {
        ostringstream msg;
        msg << mc_name << " terminated normally";

        s_num_threads_running_mutex.lock();
          --s_num_threads_running;
          msg << " [" << s_num_threads_running << "]";
          message("terminate",msg.str());
        s_num_threads_running_mutex.unlock();
        return true;
      }
    }
    else {
      return false;
    }
  }
  bool thread::keep_running() {
    m_keep_running_mutex.lock();
      bool ret = m_keep_running;
    m_keep_running_mutex.unlock();
    return ret;
  }
  void thread::execute() {
    // Implement custom thread loop function in a derived class.

    // IMPORTANT: Care must be taken with classes derived from this object. In particular, make 
    // sure to call the terminate() function in the derived class's destructor so that the 
    // execute() function returns prior to the destruction of derived members. If execute() is not 
    // terminated prior to the destruction of derived members, execute() may attempt to access an 
    // object that no longer exists.
  }
  void thread::run() {
    execute();
  }
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
