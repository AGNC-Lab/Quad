////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/globals.h                                                                   //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_GLOBALS_H
#define UTILITIES_GLOBALS_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "eigen/Core"
#include "eigen/Dense"
#include "utilities/constants.h"
#include <assert.h>
#include <errno.h>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <math.h>
#include <ostream>
#include <queue>
#include <sstream>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <typeinfo>
#include <vector>

////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UNUSED
  #if (POSIX_ENV)
    #define UNUSED(x) (void)(x);
  #elif (QT_ENV)
    #define UNUSED(x) Q_UNUSED(x)
  #endif
#endif

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;

typedef Eigen::Matrix<float,1,2> v1x2;
typedef Eigen::Matrix<float,1,3> v1x3;
typedef Eigen::Matrix<float,2,1> v2x1;
typedef Eigen::Matrix<float,3,1> v3x1;
typedef Eigen::Matrix<float,4,1> v4x1;
typedef Eigen::Matrix<float,5,1> v5x1;
typedef Eigen::Matrix<float,6,1> v6x1;
typedef Eigen::Matrix<float,7,1> v7x1;
typedef Eigen::Matrix<float,8,1> v8x1;
typedef Eigen::Matrix<float,2,2> m2x2;
typedef Eigen::Matrix<float,3,3> m3x3;
typedef Eigen::Matrix<float,3,4> m3x4;
typedef Eigen::Matrix<float,3,6> m3x6;
typedef Eigen::Matrix<float,4,3> m4x3;
typedef Eigen::Matrix<float,4,4> m4x4;
typedef Eigen::Matrix<float,6,3> m6x3;
typedef Eigen::Matrix<float,6,4> m6x4;
typedef Eigen::Matrix<float,6,6> m6x6;
typedef Eigen::Matrix<float,7,7> m7x7;
typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> mDxD;

typedef Eigen::Map<v1x2> mv1x2;
typedef Eigen::Map<v1x3> mv1x3;
typedef Eigen::Map<v2x1> mv2x1;
typedef Eigen::Map<v3x1> mv3x1;
typedef Eigen::Map<v4x1> mv4x1;
typedef Eigen::Map<v5x1> mv5x1;
typedef Eigen::Map<v6x1> mv6x1;
typedef Eigen::Map<v7x1> mv7x1;
typedef Eigen::Map<v8x1> mv8x1;
typedef Eigen::Map<m2x2> mm2x2;
typedef Eigen::Map<m3x3> mm3x3;
typedef Eigen::Map<m3x4> mm3x4;
typedef Eigen::Map<m3x6> mm3x6;
typedef Eigen::Map<m4x3> mm4x3;
typedef Eigen::Map<m4x4> mm4x4;
typedef Eigen::Map<m6x3> mm6x3;
typedef Eigen::Map<m6x4> mm6x4;
typedef Eigen::Map<m6x6> mm6x6;
typedef Eigen::Map<m7x7> mm7x7;

typedef Eigen::Matrix<double,1,2> v1x2d;
typedef Eigen::Matrix<double,1,3> v1x3d;
typedef Eigen::Matrix<double,2,1> v2x1d;
typedef Eigen::Matrix<double,3,1> v3x1d;
typedef Eigen::Matrix<double,4,1> v4x1d;
typedef Eigen::Matrix<double,5,1> v5x1d;
typedef Eigen::Matrix<double,6,1> v6x1d;
typedef Eigen::Matrix<double,7,1> v7x1d;
typedef Eigen::Matrix<double,8,1> v8x1d;
typedef Eigen::Matrix<double,2,2> m2x2d;
typedef Eigen::Matrix<double,3,3> m3x3d;
typedef Eigen::Matrix<double,3,4> m3x4d;
typedef Eigen::Matrix<double,3,6> m3x6d;
typedef Eigen::Matrix<double,4,3> m4x3d;
typedef Eigen::Matrix<double,4,4> m4x4d;
typedef Eigen::Matrix<double,6,3> m6x3d;
typedef Eigen::Matrix<double,6,4> m6x4d;
typedef Eigen::Matrix<double,6,6> m6x6d;
typedef Eigen::Matrix<double,7,7> m7x7d;
typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> mDxDd;

namespace utilities {
  void print(const std::ostringstream& msg,
             const std::string& delim="   ",
             std::ostream& stream=std::cout);
  void message(const std::string& fcn_name,
               const std::string& msg,
               std::ostream& stream=std::cout);
  void warning(const std::string& fcn_name,
               const std::string& msg,
               std::ostream& stream=std::cout);
  void error(const std::string& fcn_name,
             const std::string& msg, uint32 errnum,
             std::ostream& stream=std::cout);

  std::vector<std::string> list_files(const std::string& dir_path);
  std::string buff2bin(const void* p_source, uint32 n_bytes);
  std::string buff2hex(const void* p_source, uint32 n_bytes);
  std::string num2bin(uint8 val);
  std::string num2hex(uint8 val);
  std::string find_first_ipv4_containing(const std::string& substring);
  std::string ipv4_to_string_ip(uint32 ip);
  std::string ipv4_to_string_port(uint16 port);
  std::string ipv4_to_string(uint32 ip, uint16 port);
  uint32 ipv4_to_uint32(uint8 a1, uint8 a2, uint8 a3, uint8 a4);
  uint32 string_to_ipv4_ip(const std::string& ip);
  uint16 string_to_ipv4_port(const std::string& port);
  void string_to_ipv4(const std::string& address, uint32& ip, uint16& port);

  float wrap_rad_360(float angle_rad);
  float wrap_rad_180(float angle_rad);
  float wrap_deg_360(float angle_deg);
  float wrap_deg_180(float angle_deg);
  float tc2float(uint8 lsb, uint8 msb);

  template <class T> T sign(T val) {
    return static_cast<T>((val >= static_cast<T>(0))-(val < static_cast<T>(0)));
  }
  template <class T> T ubound(T value, T max_value) {
    return static_cast<T>(value > max_value)*max_value
          +static_cast<T>(value <= max_value)*value;
  }
  template <class T> T bound(T min_value, T value, T max_value) {
    return static_cast<T>(value < min_value)*min_value
          +static_cast<T>(value > max_value)*max_value
          +static_cast<T>((min_value <= value)&&(value <= max_value))*value;
  }
  template <class T> T lbound(T min_value, T value) {
    return static_cast<T>(value < min_value)*min_value
          +static_cast<T>(min_value <= value)*value;
  }

  template <class T> Eigen::Matrix<T,3,3> skew3(const Eigen::Matrix<T,3,1>& x) {
    return (Eigen::Matrix<T,3,3>() << 0.0,-x(2), x(1),
                                     x(2),  0.0,-x(0),
                                    -x(1), x(0),  0.0).finished();
  }
  template <class T> Eigen::Matrix<T,4,4> skew4(const Eigen::Matrix<T,3,1>& x) {
    return (Eigen::Matrix<T,4,4>() << 0.0,-x(0),-x(1),-x(2),
                                     x(0),  0.0, x(2),-x(1),
                                     x(1),-x(2),  0.0, x(0),
                                     x(2), x(1),-x(0),  0.0).finished();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
