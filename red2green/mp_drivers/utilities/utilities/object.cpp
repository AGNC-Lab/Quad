////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/object.cpp                                                                  //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/object.h"
using std::ostream;
using std::ostringstream;
using std::string;
using utilities::object;

////////////////////////////////////////////////////////////////////////////////////////////////////

object::object(const string& scope_name, const string& class_name) :
  mc_scope(scope_name),
  mc_class_name(class_name),
  mc_full_scope(mc_scope+"::"+mc_class_name+"::") {
  // Do nothing.
}
object::~object() {
  // Do nothing.
}
const string& object::scope() const {
  return mc_scope;
}
const string& object::class_name() const {
  return mc_class_name;
}
void object::print(const ostringstream& msg,
                   const string& delim,
                   ostream& stream) const {
  utilities::print(msg,delim,stream);
}
void object::message(const string& fcn_name,
                     const string& msg,
                     ostream& stream) const {
  utilities::message(mc_full_scope+fcn_name,msg,stream);
}
void object::warning(const string& fcn_name,
                     const string& msg,
                     ostream& stream) const {
  utilities::warning(mc_full_scope+fcn_name,msg,stream);
}
void object::error(const string& fcn_name,
                   const string& msg,
                   uint32 errnum,
                   ostream& stream) const {
  utilities::error(mc_full_scope+fcn_name,msg,errnum,stream);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
