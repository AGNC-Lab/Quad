////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/ifile.cpp                                                                   //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/ifile.h"
#include <algorithm>
#include <iostream>
using std::ostringstream;
using std::pair;
using std::remove;
using std::string;
using std::transform;
using utilities::ifile;

////////////////////////////////////////////////////////////////////////////////////////////////////

ifile::ifile(const string& file_name) : object("utilities","ifile") {
  // Store file name.
  m_file_name = file_name;

  // Attempt to open input file stream.
  m_file_handle.open(file_name.c_str(),std::ifstream::in);

  // Check if file was opened successfully.
  if (!m_file_handle.is_open()) {
    warning("ifile","failed to open "+m_file_name);
    return;
  }

  while (m_file_handle.good()) {
    // Initialize parameter_name and parameter_value strings.
    string new_line;
    string parameter_name;
    string parameter_value;

    // Read new line.
    char buff[500]={};
    m_file_handle.getline(buff,500,'\n');
    new_line = buff;

    // Initialize string iterator.
    string::iterator it;

    // Remove spaces from new_line string.
    it = remove(new_line.begin(),new_line.end(),0x20);
    new_line.erase(it,new_line.end());

    // Remove tabs from new_line string.
    it = remove(new_line.begin(),new_line.end(),0x09);
    new_line.erase(it,new_line.end());

    // Remove new line characters from new_line string.
    it = remove(new_line.begin(),new_line.end(),0x0A);
    new_line.erase(it,new_line.end());

    // Remove carriage return characters from new_line string.
    it = remove(new_line.begin(),new_line.end(),0x0D);
    new_line.erase(it,new_line.end());

    // Split new_line into parameter_name and parameter_value strings.
    uint32 i_delim = new_line.find_first_of("=");
    if (i_delim == string::npos) {
      // If "=" delimeter is not found, skip line.
      continue;
    }
    else {
      // If "=" delimeter is found, split string into name and value strings.
      parameter_name = new_line.substr(0,i_delim);
      parameter_value = new_line.substr(i_delim+1);
    }

    // Check if bad/fail/eof bit flag were set.
    if (m_file_handle.bad()) {
      warning("ifile",m_file_name+" set badbit flag");
      return;
    }
    else if (m_file_handle.fail()) {
      warning("ifile",m_file_name+" set failbit flag");
      return;
    }

    // Insert parameter_name and parameter_value into m_map.
    pair<map_it,bool> ret = m_map.insert(map_pair(parameter_name,parameter_value));

    // If key already existed in m_map, then print warning.
    if (ret.second == false) {
      warning("ifile","multiple instances of '"+parameter_name+"' found in "+m_file_name);
    }
  }
}
ifile::~ifile() {
  m_file_handle.close();
}
void ifile::print() const {
  message("print","beginning of contents of "+m_file_name);
  int i=1;
  for (const_map_it it=m_map.begin(); it!=m_map.end(); ++it,++i) {
    ostringstream ss;
    ss << "* " << i << " " << it->first.c_str() << " = " << it->second.c_str() << std::endl;
    utilities::print(ss);
  }
  message("print","end of contents of "+m_file_name);
}
bool ifile::is_open() const {
  return m_file_handle.is_open();
}
bool ifile::read_string(const string& parameter_name, string& value) const {
    if (m_file_handle.is_open()) {
    // Search for parameter_name in m_map.
    const_map_it it = m_map.find(parameter_name);

    // Enter this branch if parameter_name is not contained in m_map.
    if (it == m_map.end()) {
      warning("read_string","'"+parameter_name+"' is not a parameter in "+m_file_name);
      value = string("");
      return false;
    }
    // Enter this branch if parameter_name is contained in m_map.
    else {
      value = it->second;
      return true;
    }
  }
  else {
    value = string("");
    return false;
  }
}
bool ifile::read_bool(const string& parameter_name, bool& value) const {
  if (m_file_handle.is_open()) {
    // Search for parameter_name in m_map.
    const_map_it it = m_map.find(parameter_name);

    // Enter this branch if parameter_name is not contained in m_map.
    if (it == m_map.end()) {
      warning("read_bool","'"+parameter_name+"' is not a parameter in "+m_file_name);
      value = false;
      return false;
    }
    // Enter this branch if parameter_name is contained in m_map.
    else {
      // Extract string from map iterator.
      string parameter_value = it->second;
      string parameter_value_lower_case = parameter_value;

      // Convert string to lower case.
      transform(parameter_value_lower_case.begin(),parameter_value_lower_case.end(),
                parameter_value_lower_case.begin(),::tolower);

      // Check if parameter_value_lower_case has a valid value.
      if (parameter_value_lower_case == "true") {
        value = true;
        return true;
      }
      else if (parameter_value_lower_case == "false") {
        value = false;
        return true;
      }
      else {
        warning("read_bool",
          "parameter '"+parameter_name+"' has invalid value '"+parameter_value+"'");
        value = false;
        return false;
      }
    }
  }
  else {
    value = false;
    return false;
  }
}
bool ifile::read_int8(const string& parameter_name, int8& value) const {
  if (m_file_handle.is_open()) {
    // Search for parameter_name in m_map.
    const_map_it it = m_map.find(parameter_name);

    // Enter this branch if parameter_name is not contained in m_map.
    if (it == m_map.end()) {
      warning("read_int8","'"+parameter_name+"' is not a parameter in "+m_file_name);
      value = 0;
      return false;
    }
    // Enter this branch if parameter_name is contained in m_map.
    else {
      string parameter_value = it->second;
      value = static_cast<int8>(atoi(parameter_value.c_str()));
      return true;
    }
  }
  else {
    value = 0;
    return false;
  }
}
bool ifile::read_int16(const string& parameter_name, int16& value) const {
  if (m_file_handle.is_open()) {
    // Search for parameter_name in m_map.
    const_map_it it = m_map.find(parameter_name);

    // Enter this branch if parameter_name is not contained in m_map.
    if (it == m_map.end()) {
      warning("read_int16","'"+parameter_name+"' is not a parameter in "+m_file_name);
      value = 0;
      return false;
    }
    // Enter this branch if parameter_name is contained in m_map.
    else {
      string parameter_value = it->second;
      value = static_cast<int16>(atoi(parameter_value.c_str()));
      return true;
    }
  }
  else {
    value = 0;
    return false;
  }
}
bool ifile::read_int32(const string& parameter_name, int32& value) const {
  if (m_file_handle.is_open()) {
    // Search for parameter_name in m_map.
    const_map_it it = m_map.find(parameter_name);

    // Enter this branch if parameter_name is not contained in m_map.
    if (it == m_map.end()) {
      warning("read_int32","'"+parameter_name+"' is not a parameter in "+m_file_name);
      value = 0;
      return false;
    }
    // Enter this branch if parameter_name is contained in m_map.
    else {
      string parameter_value = it->second;
      value = static_cast<int32>(atoi(parameter_value.c_str()));
      return true;
    }
  }
  else {
    value = 0;
    return false;
  }
}
bool ifile::read_uint8(const string& parameter_name, uint8& value) const {
  if (m_file_handle.is_open()) {
    // Search for parameter_name in m_map.
    const_map_it it = m_map.find(parameter_name);

    // Enter this branch if parameter_name is not contained in m_map.
    if (it == m_map.end()) {
      warning("read_uint8","'"+parameter_name+"' is not a parameter in "+m_file_name);
      value = 0;
      return false;
    }
    // Enter this branch if parameter_name is contained in m_map.
    else {
      string parameter_value = it->second;
      value = static_cast<uint8>(atoi(parameter_value.c_str()));
      return true;
    }
  }
  else {
    value = 0;
    return false;
  }
}
bool ifile::read_uint16(const string& parameter_name, uint16& value) const {
  if (m_file_handle.is_open()) {
    // Search for parameter_name in m_map.
    const_map_it it = m_map.find(parameter_name);

    // Enter this branch if parameter_name is not contained in m_map.
    if (it == m_map.end()) {
      warning("read_uint16","'"+parameter_name+"' is not a parameter in "+m_file_name);
      value = 0;
      return false;
    }
    // Enter this branch if parameter_name is contained in m_map.
    else {
      string parameter_value = it->second;
      value = static_cast<uint16>(atoi(parameter_value.c_str()));
      return true;
    }
  }
  else {
    value = 0;
    return false;
  }
}
bool ifile::read_uint32(const string& parameter_name, uint32& value) const {
  if (m_file_handle.is_open()) {
    // Search for parameter_name in m_map.
    const_map_it it = m_map.find(parameter_name);

    // Enter this branch if parameter_name is not contained in m_map.
    if (it == m_map.end()) {
      warning("read_uint32","'"+parameter_name+"' is not a parameter in "+m_file_name);
      value = 0;
      return false;
    }
    // Enter this branch if parameter_name is contained in m_map.
    else {
      string parameter_value = it->second;
      value = static_cast<uint32>(atoi(parameter_value.c_str()));
      return true;
    }
  }
  else {
    value = 0;
    return false;
  }
}
bool ifile::read_float(const string& parameter_name, float& value) const {
  if (m_file_handle.is_open()) {
    // Search for parameter_name in m_map.
    const_map_it it = m_map.find(parameter_name);

    // Enter this branch if parameter_name is not contained in m_map.
    if (it == m_map.end()) {
      warning("read_float","'"+parameter_name+"' is not a parameter in "+m_file_name);
      value = 0.0f;
      return false;
    }
    // Enter this branch if parameter_name is contained in m_map.
    else {
      string parameter_value = it->second;
      value = static_cast<float>(atof(parameter_value.c_str()));
      return true;
    }
  }
  else {
    value = 0.0f;
    return false;
  }
}
bool ifile::read_double(const string& parameter_name, double& value) const {
  if (m_file_handle.is_open()) {
    // Search for parameter_name in m_map.
    const_map_it it = m_map.find(parameter_name);

    // Enter this branch if parameter_name is not contained in m_map.
    if (it == m_map.end()) {
      warning("read_double","'"+parameter_name+"' is not a parameter in "+m_file_name);
      value = 0.0;
      return false;
    }
    // Enter this branch if parameter_name is contained in m_map.
    else {
      string parameter_value = it->second;
      value = atof(parameter_value.c_str());
      return true;
    }
  }
  else {
    value = 0.0;
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
