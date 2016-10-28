////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/tob.cpp                                                                     //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/tob.h"
using std::ostream;
using utilities::tob;

////////////////////////////////////////////////////////////////////////////////////////////////////

tob tob::time() {
  return tob(SYNC);
}
tob tob::sec2tob(float t_sec) {
  tob t;
  uint64 nsec = (uint64)(t_sec*1.0e9f);
  t.m_t.tv_sec = (uint32)(nsec/1000000000);
  t.m_t.tv_nsec = (uint32)(nsec%1000000000);
  return t;
}
tob tob::msec2tob(uint32 t_msec) {
  tob t;
  t.m_t.tv_sec = t_msec/1000;
  t.m_t.tv_nsec = t_msec%1000*1000000;
  return t;
}
tob tob::usec2tob(uint32 t_usec) {
  tob t;
  t.m_t.tv_sec = t_usec/1000000;
  t.m_t.tv_nsec = t_usec%1000000*1000;
  return t;
}
tob tob::nsec2tob(uint32 t_nsec) {
  tob t;
  t.m_t.tv_sec = t_nsec/1000000000;
  t.m_t.tv_nsec = t_nsec%1000000000;
  return t;
}

tob::tob(MODE mode) {
  switch (mode) {
    case DONT_SYNC:
      m_t.tv_sec = 0;
      m_t.tv_nsec = 0;
      break;
    case SYNC:
      sync_to_clock();
      break;
  }
}
tob::tob(uint32 sec, uint32 nsec) {
  m_t.tv_sec = sec;
  m_t.tv_nsec = nsec;
}
tob::tob(const struct timespec& t) {
  m_t.tv_sec = t.tv_sec;
  m_t.tv_nsec = t.tv_nsec;
}
int32 tob::sec() const {
  return m_t.tv_sec;
}
int32 tob::nsec() const {
  return m_t.tv_nsec;
}
void tob::to_hms(uint32& h, uint32& m, float& s) const {
  // This function assumes that m_t.tv_sec > 0, m_t.tv_nsec > 0, and m_t.tv_nsec < 1000000000.
  h = m_t.tv_sec/3600;
  m = m_t.tv_sec%3600/60;
  s = (float)(m_t.tv_sec%60)+1e-9f*(float)(m_t.tv_nsec);
}
float tob::to_sec() const {
  return ((float)m_t.tv_sec+(1.0e-9f*(float)m_t.tv_nsec));
}
int32 tob::to_msec() const {
  return (int32)(1000*m_t.tv_sec+m_t.tv_nsec/1000000);
}
int32 tob::to_usec() const {
  return (int32)(1000000*m_t.tv_sec+m_t.tv_nsec/1000);
}
int32 tob::to_nsec() const {
  return (int32)(1000000000*m_t.tv_sec+m_t.tv_nsec);
}
tob& tob::from_sec(float t_sec) {
  uint64 nsec = (uint64)(t_sec*1.0e9f);
  m_t.tv_sec = (uint32)(nsec/1000000000);
  m_t.tv_nsec = (uint32)(nsec%1000000000);
  return *this;
}
tob& tob::from_msec(uint32 t_msec) {
  m_t.tv_sec = t_msec/1000;
  m_t.tv_nsec = t_msec%1000*1000000;
  return *this;
}
tob& tob::from_usec(uint32 t_usec) {
  m_t.tv_sec = t_usec/1000000;
  m_t.tv_nsec = t_usec%1000000*1000;
  return *this;
}
tob& tob::from_nsec(uint32 t_nsec) {
  m_t.tv_sec = t_nsec/1000000000;
  m_t.tv_nsec = t_nsec%1000000000;
  return *this;
}
const struct timespec* tob::to_timespec() const {
  return &m_t;
}
tob& tob::sync_to_clock() {
  #if (POSIX_ENV)
    clock_gettime(CLOCK_REALTIME,&m_t);
  #elif (QT_ENV)
    uint64 t_msec = (uint64)QDateTime::currentMSecsSinceEpoch();
    m_t.tv_sec = t_msec/1000;
    m_t.tv_nsec = t_msec%1000*1000000;
  #endif
  return *this;
}
tob& tob::operator=(const tob& t) {
  m_t = t.m_t;
  return *this;
}
void tob::operator+=(const tob& t) {
  m_t.tv_sec += t.m_t.tv_sec;
  m_t.tv_nsec += t.m_t.tv_nsec;

  m_t.tv_sec += m_t.tv_nsec/1000000000;
  m_t.tv_nsec %= 1000000000;
}
void tob::operator-=(const tob& t) {
  m_t.tv_sec -= t.m_t.tv_sec;
  m_t.tv_nsec -= t.m_t.tv_nsec;

  m_t.tv_sec += m_t.tv_nsec/-1000000000;
  m_t.tv_nsec %= 1000000000;

  // This part is required to ensure that tv_nsec is not less than zero. For example, 0.9 can be
  // represented as (tv_sec,tv_nsec) = (0,900000000) (correct) or (tv_sec,tv_nsec) = (1,-100000000) 
  // (incorrect form for nanosleep()).
  int64 nsec = ((int64)(m_t.tv_sec))*1000000000+(int64)(m_t.tv_nsec);
  m_t.tv_sec = (time_t)(nsec/1000000000);
  m_t.tv_nsec = (long int)(nsec%1000000000);
}
tob tob::operator+(const tob& t) const {
  tob res;

  res.m_t.tv_sec = m_t.tv_sec+t.m_t.tv_sec;
  res.m_t.tv_nsec = m_t.tv_nsec+t.m_t.tv_nsec;

  res.m_t.tv_sec += res.m_t.tv_nsec/1000000000;
  res.m_t.tv_nsec %= 1000000000;

  return res;
}
tob tob::operator-(const tob& t) const {
  tob res;

  res.m_t.tv_sec = m_t.tv_sec-t.m_t.tv_sec;
  res.m_t.tv_nsec = m_t.tv_nsec-t.m_t.tv_nsec;

  res.m_t.tv_sec += res.m_t.tv_nsec/-1000000000;
  res.m_t.tv_nsec %= 1000000000;

  // This part is required to ensure that tv_nsec is not less than zero. For example, 0.9 can be
  // represented as (tv_sec,tv_nsec) = (0,900000000) (correct) or (tv_sec,tv_nsec) = (1,-100000000) 
  // (incorrect form for nanosleep()).
  int64 nsec = ((int64)(res.m_t.tv_sec))*1000000000+(int64)(res.m_t.tv_nsec);
  res.m_t.tv_sec = (time_t)(nsec/1000000000);
  res.m_t.tv_nsec = (long int)(nsec%1000000000);

  return res;
}
bool tob::operator>(const tob& t) const {
  // Here we assume that tv_sec >= 0 and tv_nsec >= 0. 
  return (m_t.tv_sec > t.m_t.tv_sec)||
    ((m_t.tv_sec == t.m_t.tv_sec)&&(m_t.tv_nsec > t.m_t.tv_nsec));
}
bool tob::operator<(const tob& t) const {
  // Here we assume that tv_sec >= 0 and tv_nsec >= 0.
  return (t.m_t.tv_sec > m_t.tv_sec)||
    ((t.m_t.tv_sec == m_t.tv_sec)&&(t.m_t.tv_nsec > t.m_t.tv_nsec));
}

namespace utilities {
  ostream& operator<<(ostream& stream, const tob& t) {
    uint32 hh;
    uint32 mm;
    float ssss;
    t.to_hms(hh,mm,ssss);

    stream << "["
           << std::fixed << std::setbase(10) << std::setfill('0') << std::setprecision(0)
           << std::setw(2) << hh << ":"
           << std::setw(2) << mm << ":"
           << std::setprecision(3)
           << std::setw(6) << ssss
           << "]";

    return stream;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
