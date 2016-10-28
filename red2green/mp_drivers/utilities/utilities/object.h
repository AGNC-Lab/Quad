////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/object.h                                                                    //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_OBJECT_H
#define UTILITIES_OBJECT_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class object {
    private:
      const std::string mc_scope;
      const std::string mc_class_name;
      const std::string mc_full_scope;

    public:
      object(const std::string& scope_name, const std::string& class_name);
      virtual ~object();
    protected:
      const std::string& scope() const;
      const std::string& class_name() const;
      void print(const std::ostringstream& msg,
                 const std::string& delim="   ",
                 std::ostream& stream=std::cout) const;
      void message(const std::string& fcn_name,
                   const std::string& msg,
                   std::ostream& stream=std::cout) const;
      void warning(const std::string& fcn_name,
                   const std::string& msg,
                   std::ostream& stream=std::cout) const;
      void error(const std::string& fcn_name,
                 const std::string& msg, uint32 errnum,
                 std::ostream& stream=std::cout) const;
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
