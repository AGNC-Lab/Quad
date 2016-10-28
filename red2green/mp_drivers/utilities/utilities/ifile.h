////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/ifile.h                                                                     //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_IFILE_H
#define UTILITIES_IFILE_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"
#include "utilities/object.h"
#include <fstream>

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  class ifile : public object {
      typedef std::map<std::string,std::string>::const_iterator const_map_it;
      typedef std::map<std::string,std::string>::iterator map_it;
      typedef std::pair<std::string,std::string> map_pair;

    private:
      std::string m_file_name;
      std::ifstream m_file_handle;
      std::map<std::string,std::string> m_map;

    public:
      ifile(const std::string& file_name);
      ~ifile();
      void print() const;
      bool is_open() const;
      bool read_string(const std::string& parameter_name, std::string& value) const;
      bool read_bool(const std::string& parameter_name, bool& value) const;
      bool read_int8(const std::string& parameter_name, int8& value) const;
      bool read_int16(const std::string& parameter_name, int16& value) const;
      bool read_int32(const std::string& parameter_name, int32& value) const;
      bool read_uint8(const std::string& parameter_name, uint8& value) const;
      bool read_uint16(const std::string& parameter_name, uint16& value) const;
      bool read_uint32(const std::string& parameter_name, uint32& value) const;
      bool read_float(const std::string& parameter_name, float& value) const;
      bool read_double(const std::string& parameter_name, double& value) const;
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
