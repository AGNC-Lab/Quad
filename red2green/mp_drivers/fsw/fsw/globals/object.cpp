////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/globals/fsw_obj.cpp                                                               //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "fsw/globals/object.h"
using fsw::globals::object;
using std::string;

////////////////////////////////////////////////////////////////////////////////////////////////////

object::object(const string& scope_name, const string& class_name) :
  utilities::object("fsw::"+scope_name,class_name) {
	// Do nothing.
}

////////////////////////////////////////////////////////////////////////////////////////////////////
