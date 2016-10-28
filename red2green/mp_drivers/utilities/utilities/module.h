////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/module.h                                                                    //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_MODULE_H
#define UTILITIES_MODULE_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"
#include "utilities/object.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class module : public object {
    // List of pure virtual functions:
    // (1) void step(float ts)

    private:
      uint32 m_exec_interval;
      uint32 m_counter;

    public:
      module(const std::string& scope_name, const std::string& class_name, uint32 exec_interval);
      virtual ~module();
      void reset_counter();
      void set_exec_interval(uint32 exec_interval);
      void execute(float ts);
    protected:
      virtual void step(float ts) = 0;
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
