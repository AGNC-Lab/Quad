////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/globals/fsw_obj.h                                                                 //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FSW_GLOBAL_FSW_OBJ_H
#define FSW_GLOBAL_FSW_OBJ_H

#include "utilities/object.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace fsw {
  namespace globals {
    class object : public utilities::object {
      public:
        object(const std::string& scope_name, const std::string& class_name);
    };
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
