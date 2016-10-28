////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/thread.h                                                                    //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_THREAD_H
#define UTILITIES_THREAD_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/globals.h"
  #include "utilities/mutex.h"
  #include "utilities/object.h"
  #include <pthread.h>
#elif (QT_ENV)
  #include "utilities/globals.h"
  #include "utilities/mutex.h"
  #include "utilities/object.h"
  #include <QThread>
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  #if (POSIX_ENV)
    class thread : public object {
      // List of pure virtual functions:
      // (1) void execute()

      public:
        enum PRIORITY {
          PR1=-20,
          PR2,
          PR3,
          PR4,
          PR5,
          PR6,
          PR7,
          PR8,
          PR9,
          PR10,
          PR11,
          PR12,
          PR13,
          PR14,
          PR15
        };

      private:
        static uint32 s_num_threads_running;
        static mutex s_num_threads_running_mutex;

      private:
        const std::string mc_name;
        const PRIORITY mc_priority;
        bool m_is_running;
        mutex m_is_running_mutex;
        pthread_t m_handle;
        bool m_keep_running;
        mutex m_keep_running_mutex;

      public:
        thread(const std::string& name, PRIORITY priority);
        virtual ~thread();
        const std::string& name() const;
        bool is_running() const;
        bool launch();
        bool terminate();
      protected:
        bool keep_running();
        virtual void execute() = 0;
      private:
        static void* thread_function(void* arg);
    };
  #elif (QT_ENV)
    class thread : public object, public QThread {
      public:
        enum PRIORITY {
          PR0=QThread::InheritPriority,
          PR1=QThread::TimeCriticalPriority,
          PR2=QThread::HighestPriority,
          PR3=QThread::NormalPriority,
          PR4=QThread::LowPriority,
          PR5=QThread::LowestPriority,
          PR6=QThread::IdlePriority
        };

      private:
        static uint32 s_num_threads_running;
        static mutex s_num_threads_running_mutex;

      private:
        std::string mc_name;
        PRIORITY mc_priority;
        bool m_keep_running;
        mutex m_keep_running_mutex;

      public:
        thread(const std::string& name, PRIORITY priority);
        virtual ~thread();
        const std::string& name() const;
        bool is_running() const;
        bool launch();
        bool terminate();
      protected:
        bool keep_running();
        virtual void execute() = 0;
      private:
        void run();
    };
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
