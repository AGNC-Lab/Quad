////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/tob.h                                                                       //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_TOB_H
#define UTILITIES_TOB_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/globals.h"
  #include <sys/time.h>
#elif (QT_ENV)
  #include "utilities/globals.h"
  #include <QDateTime>
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class tob {
    public:
      enum MODE{DONT_SYNC,SYNC};

    public:
      static tob time();
      static tob sec2tob(float t_sec);
      static tob msec2tob(uint32 t_msec);
      static tob usec2tob(uint32 t_usec);
      static tob nsec2tob(uint32 t_nsec);

    private:
      struct timespec m_t;

    public:
      tob(MODE mode=DONT_SYNC);
      tob(uint32 sec, uint32 nsec);
      tob(const struct timespec& t);
      int32 sec() const;
      int32 nsec() const;
      void to_hms(uint32& h, uint32& m, float& s) const;
      float to_sec() const;
      int32 to_msec() const;
      int32 to_usec() const;
      int32 to_nsec() const;
      tob& from_sec(float t_sec);
      tob& from_msec(uint32 t_msec);
      tob& from_usec(uint32 t_usec);
      tob& from_nsec(uint32 t_nsec);
      const struct timespec* to_timespec() const;
      tob& sync_to_clock();
      tob& operator =(const tob& t);
      void operator+=(const tob& t);
      void operator-=(const tob& t);
      tob operator+(const tob& t) const;
      tob operator-(const tob& t) const;
      bool operator>(const tob& t) const;
      bool operator<(const tob& t) const;

    friend std::ostream& operator<<(std::ostream& stream, const tob& t);
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
