////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/globals/fsw_module.h                                                              //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FSW_GLOBAL_FSW_MODULE_H
#define FSW_GLOBAL_FSW_MODULE_H

#include "utilities/module.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace fsw {
  namespace globals {
    class module : public utilities::module {
      // List of pure virtual functions:
      // (1) void step(float ts)
      
      public:
        module(const std::string& scope_name, const std::string& class_name, uint32 exec_count);
    };
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
