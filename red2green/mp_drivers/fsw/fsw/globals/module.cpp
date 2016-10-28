////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/globals/fsw_module.cpp                                                            //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "fsw/globals/module.h"
using fsw::globals::module;
using std::string;

////////////////////////////////////////////////////////////////////////////////////////////////////

module::module(const string& scope_name, const string& class_name, uint32 exec_count) :
	utilities::module("fsw::"+scope_name,class_name,exec_count) {
	// Do nothing.
}

////////////////////////////////////////////////////////////////////////////////////////////////////
