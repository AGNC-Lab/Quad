////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/ofile.h                                                                     //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_OFILE_H
#define UTILITIES_OFILE_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"
#include "utilities/object.h"
#include <fstream>

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class ofile : public object {
    public:
      enum MODE {
        APPEND,
        OVERWRITE
      };

    private:
      std::string m_file_name;
      std::ofstream m_file_handle;

    public:
      ofile(const std::string& file_name, MODE mode);
      ~ofile();
      bool is_open() const;
      bool write_bool(const std::string& parameter_name, bool value);
      bool write_int8(const std::string& parameter_name, int8 value);
      bool write_int16(const std::string& parameter_name, int16 value);
      bool write_int32(const std::string& parameter_name, int32 value);
      bool write_uint8(const std::string& parameter_name, uint8 value);
      bool write_uint16(const std::string& parameter_name, uint16 value);
      bool write_uint32(const std::string& parameter_name, uint32 value);
      bool write_float(const std::string& parameter_name, float value);
      bool write_double(const std::string& parameter_name, double value);
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
