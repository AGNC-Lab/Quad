////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/ofile.cpp                                                                   //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/ofile.h"
using std::string;
using utilities::ofile;

////////////////////////////////////////////////////////////////////////////////////////////////////

ofile::ofile(const string& file_name, MODE mode) : object("utilities","ofile") {
  // Store file name.
  m_file_name = file_name;

  // Attempt to open output file stream.
  switch (mode) {
    case APPEND:
      m_file_handle.open(file_name.c_str(),std::ifstream::app);
      break;
    case OVERWRITE:
      m_file_handle.open(file_name.c_str(),std::ifstream::trunc);
      break;
  }

  // Check if file was opened successfully.
  if (!m_file_handle.is_open()) {
    warning("ofile","failed to open "+m_file_name);
    return;
  }
}
ofile::~ofile() {
  m_file_handle.close();
}
bool ofile::is_open() const {
  return m_file_handle.is_open();
}
bool ofile::write_bool(const string& parameter_name, bool value) {
  // Write line to file.
  if (value == true) {
    m_file_handle << parameter_name << " = true\n";
  }
  else {
    m_file_handle << parameter_name << " = false\n";
  }

  // Flush to file.
  m_file_handle.flush();

  // Check if fail bit flag was set. If set, return false.
  if (m_file_handle.fail()) {
    warning("write_bool",m_file_name+" set failbit flag");
    return false;
  }

  // Check if bad bit flag was set. If set, return false.
  if (m_file_handle.bad()) {
    warning("write_bool",m_file_name+" set badbit flag");
    return false;
  }

  // Return true on success.
  return true;
}
bool ofile::write_int8(const string& parameter_name, int8 value) {
  // Write line to file.
  m_file_handle << parameter_name << " = " << value << "\n";

  // Flush to file.
  m_file_handle.flush();

  // Check if fail bit flag was set. If set, return false.
  if (m_file_handle.fail()) {
    warning("write_int8",m_file_name+" set failbit flag");
    return false;
  }

  // Check if bad bit flag was set. If set, return false.
  if (m_file_handle.bad()) {
    warning("write_int8",m_file_name+" set badbit flag");
    return false;
  }

  // Return true on success. 
  return true;
}
bool ofile::write_int16(const string& parameter_name, int16 value) {
  // Write line to file.
  m_file_handle << parameter_name << " = " << value << "\n";

  // Flush to file.
  m_file_handle.flush();

  // Check if fail bit flag was set. If set, return false.
  if (m_file_handle.fail()) {
    warning("write_int16",m_file_name+" set failbit flag");
    return false;
  }

  // Check if bad bit flag was set. If set, return false.
  if (m_file_handle.bad()) {
    warning("write_int16",m_file_name+" set badbit flag");
    return false;
  }

  // Return true on success. 
  return true;
}
bool ofile::write_int32(const string& parameter_name, int32 value) {
  // Write line to file.
  m_file_handle << parameter_name << " = " << value << "\n";

  // Flush to file.
  m_file_handle.flush();

  // Check if fail bit flag was set. If set, return false.
  if (m_file_handle.fail()) {
    warning("write_int32",m_file_name+" set failbit flag");
    return false;
  }

  // Check if bad bit flag was set. If set, return false.
  if (m_file_handle.bad()) {
    warning("write_int32",m_file_name+" set badbit flag");
    return false;
  }

  // Return true on success. 
  return true;
}
bool ofile::write_uint8(const string& parameter_name, uint8 value) {
  // Write line to file.
  m_file_handle << parameter_name << " = " << value << "\n";

  // Flush to file.
  m_file_handle.flush();

  // Check if fail bit flag was set. If set, return false.
  if (m_file_handle.fail()) {
    warning("write_uint8",m_file_name+" set failbit flag");
    return false;
  }

  // Check if bad bit flag was set. If set, return false.
  if (m_file_handle.bad()) {
    warning("write_uint8",m_file_name+" set badbit flag");
    return false;
  }

  // Return true on success. 
  return true;
}
bool ofile::write_uint16(const string& parameter_name, uint16 value) {
  // Write line to file.
  m_file_handle << parameter_name << " = " << value << "\n";

  // Flush to file.
  m_file_handle.flush();

  // Check if fail bit flag was set. If set, return false.
  if (m_file_handle.fail()) {
    warning("write_uint16",m_file_name+" set failbit flag");
    return false;
  }

  // Check if bad bit flag was set. If set, return false.
  if (m_file_handle.bad()) {
    warning("write_uint16",m_file_name+" set badbit flag");
    return false;
  }

  // Return true on success. 
  return true;
}
bool ofile::write_uint32(const string& parameter_name, uint32 value) {
  // Write line to file.
  m_file_handle << parameter_name << " = " << value << "\n";

  // Flush to file.
  m_file_handle.flush();

  // Check if fail bit flag was set. If set, return false.
  if (m_file_handle.fail()) {
    warning("write_uint32",m_file_name+" set failbit flag");
    return false;
  }

  // Check if bad bit flag was set. If set, return false.
  if (m_file_handle.bad()) {
    warning("write_uint32",m_file_name+" set badbit flag");
    return false;
  }

  // Return true on success. 
  return true;
}
bool ofile::write_float(const string& parameter_name, float value) {
  // Write line to file.
  m_file_handle << parameter_name << " = " << value << "\n";

  // Flush to file.
  m_file_handle.flush();

  // Check if fail bit flag was set. If set, return false.
  if (m_file_handle.fail()) {
    warning("write_float",m_file_name+" set failbit flag");
    return false;
  }

  // Check if bad bit flag was set. If set, return false.
  if (m_file_handle.bad()) {
    warning("write_float",m_file_name+" set badbit flag");
    return false;
  }

  // Return true on success.
  return false;
}
bool ofile::write_double(const string& parameter_name, double value) {
  // Write line to file.
  m_file_handle << parameter_name << " = " << value << "\n";

  // Flush to file.
  m_file_handle.flush();

  // Check if fail bit flag was set. If set, return false.
  if (m_file_handle.fail()) {
    warning("write_double",m_file_name+" set failbit flag");
    return false;
  }

  // Check if bad bit flag was set. If set, return false.
  if (m_file_handle.bad()) {
    warning("write_double",m_file_name+" set badbit flag");
    return false;
  }

  // Return true on success.
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
