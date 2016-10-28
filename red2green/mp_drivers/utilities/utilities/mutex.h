////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/mutex.h                                                                     //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_MUTEX_H
#define UTILITIES_MUTEX_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/globals.h"
  #include "utilities/object.h"
  #include <pthread.h>
#elif (QT_ENV)
  #include "utilities/globals.h"
  #include "utilities/object.h"
  #include <QMutex>
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class mutex : public object {
    private:
      #if (POSIX_ENV)
        pthread_mutex_t m_mutex;
        pthread_mutex_t* mp_mutex;
      #elif (QT_ENV)
        QMutex m_mutex;
        QMutex* mp_mutex;
      #endif

    public:
      mutex();
      ~mutex();
      void lock() const;
      bool try_lock() const;
      void unlock() const;
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
