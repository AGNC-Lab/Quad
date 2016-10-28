////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/timer.h                                                                     //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_TIMER_H
#define UTILITIES_TIMER_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"
#include "utilities/mutex.h"
#include "utilities/object.h"
#include "utilities/tob.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class timer : public object {
    private:
      static tob s_epoch;

    public:
      static void reset_epoch();
      static tob get_timestamp();
      static void wait(const uint64& t_nsec);
      static void wait(const tob& t);

    private:
      std::string m_name;
      tob m_period;
      tob m_initial;
      tob m_target;

    public:
      timer(const std::string& name, uint32 period_msec=0);
      void reset();
      timer& await_target();
      tob time_elapsed() const;
      const tob& period() const;
      const tob& initial() const;
      const tob& target() const;
      timer& operator++();
      timer& operator--();
      timer operator++(int);
      timer operator--(int);
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
