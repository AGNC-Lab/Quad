////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/globals/global_defs.h                                                             //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FSW_GLOBAL_GLOBALS_H
#define FSW_GLOBAL_GLOBALS_H

#include "utilities/globals.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace fsw {
  namespace globals {
    class version {
      public:
        static const std::string string;
    };
    
    void preamble();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
