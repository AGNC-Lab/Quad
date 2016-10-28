////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/timer.cpp                                                                   //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/timer.h"
#elif (QT_ENV)
  #include "utilities/timer.h"
  #include <QThread>
#endif
using std::ostringstream;
using std::string;
using utilities::message;
using utilities::timer;
using utilities::tob;
using utilities::warning;

////////////////////////////////////////////////////////////////////////////////////////////////////

tob timer::s_epoch(tob::SYNC);

void timer::reset_epoch() {
  s_epoch.sync_to_clock();
}
tob timer::get_timestamp() {
  tob current;
  current.sync_to_clock();
  return current-s_epoch;
}
void timer::wait(const uint64& t_nsec) {
  #if (POSIX_ENV)
    struct timespec t;
    t.tv_sec = t_nsec/1000000000;
    t.tv_nsec = t_nsec%1000000000;
    nanosleep(&t,NULL);
  #elif (QT_ENV)
    uint32 t_usec = (uint32)(t_nsec/1000);
    QThread::usleep(t_usec);
  #endif
}
void timer::wait(const tob& t) {
  #if (POSIX_ENV)
    nanosleep(t.to_timespec(),NULL);
  #elif (QT_ENV)
    uint32 t_usec = (uint32)t.to_usec();
    QThread::usleep(t_usec);
  #endif
}

timer::timer(const string& name, uint32 period_msec) : object("utilities","timer") {
  m_name = name;
  m_period.from_msec(period_msec);

  m_initial.sync_to_clock();
  m_target = m_initial;

  if (period_msec > 0) {
    ostringstream msg;
    msg << m_name << " clock set to ";
    msg.precision(4);
    msg << 1.0f/m_period.to_sec() << " [Hz]";
    message("timer",msg.str());
  }
}
void timer::reset() {
  // Resetting timer sets the target time to one period ahead of the current time. This is done 
  // in order to avoid cycle slips right after clock reset.
  m_initial.sync_to_clock();
  m_target = m_initial+m_period;
  message("reset","resetting "+m_name+" clock");
}
timer& timer::await_target() {
  tob current;
  current.sync_to_clock();

  if (m_target > current) {
    wait(m_target-current);
  }
  #if !defined(SUPPRESS_TIMER_SLIP_WARNINGS)
    else {
      ostringstream msg;
      msg << m_name << " clock slipped " << -(m_target-current).to_usec() << " [us]";
      warning("await_target",msg.str());
    }
  #endif

  return *this;
}
tob timer::time_elapsed() const {
  tob current;
  current.sync_to_clock();
  return current-m_initial;
}
const tob& timer::period() const {
  return m_period;
}
const tob& timer::initial() const {
  return m_initial;
}
const tob& timer::target() const {
  return m_target;
}
timer& timer::operator++() {
  m_target += m_period;
  return *this;
}
timer& timer::operator--() {
  m_target -= m_period;
  return *this;
}
timer timer::operator++(int) {
  timer copy = *this;
  m_target += m_period;
  return copy;
}
timer timer::operator--(int) {
  timer copy = *this;
  m_target -= m_period;
  return copy;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
