////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/module.cpp                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/module.h"
using std::ostream;
using std::ostringstream;
using std::string;
using utilities::module;

////////////////////////////////////////////////////////////////////////////////////////////////////

module::module(const string& scope_name, const string& class_name, uint32 exec_interval) :
  object(scope_name,class_name) {
  set_exec_interval(exec_interval);
}
module::~module() {
  // Do nothing.
}
void module::reset_counter() {
  m_counter = 0;
}
void module::set_exec_interval(uint32 exec_interval) {
  m_exec_interval = exec_interval;
}
void module::execute(float ts) {
  if ((m_counter++ % m_exec_interval) == 0) {
    step(ts);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
