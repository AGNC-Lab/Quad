////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/cv.h                                                                        //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_CV_H
#define UTILITIES_CV_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/globals.h"
  #include "utilities/object.h"
  #include <pthread.h>
#elif (QT_ENV)
  #include "utilities/globals.h"
  #include "utilities/object.h"
  #include <QMutex>
  #include <QWaitCondition>
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class cv : public object {
    private:
      #if (POSIX_ENV)
        pthread_cond_t m_cond_var;
        pthread_mutex_t m_mutex;
      #elif (QT_ENV)
        QWaitCondition m_cond_var;
        QMutex m_mutex;
      #endif
      bool m_wait_condition;

    public:
      cv();
      ~cv();
      void wait();
      void signal();
      void broadcast();
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
