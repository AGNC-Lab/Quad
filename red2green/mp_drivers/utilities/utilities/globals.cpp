////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/globals.cpp                                                                 //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#if (POSIX_ENV)
  #include "utilities/globals.h"
  #include "utilities/mutex.h"
  #include "utilities/timer.h"
  #include <arpa/inet.h>
  #include <dirent.h>
  #include <ifaddrs.h>
  #include <netinet/in.h>
#elif (QT_ENV)
  #include "utilities/globals.h"
  #include "utilities/mutex.h"
  #include "utilities/timer.h"
  #include <QDebug>
  #include <QNetworkInterface>
#endif
using constants::pi;
using std::ostream;
using std::ostringstream;
using std::string;
using std::vector;
using utilities::mutex;
using utilities::timer;

////////////////////////////////////////////////////////////////////////////////////////////////////

void utilities::print(const ostringstream& msg, const string& delim, ostream& stream) {
  static mutex s_mutex;

  ostringstream ss;
  ss << timer::get_timestamp() << delim << msg.str();

  s_mutex.lock();
    #if (POSIX_ENV)
      stream << ss.str() << std::endl;
    #elif (QT_ENV)
      UNUSED(stream);
      qDebug() << ss.str().c_str();
    #endif
  s_mutex.unlock();
}
void utilities::message(const string& fcn_name, const string& msg, ostream& stream) {
  ostringstream ss;
  ss << fcn_name << "() - " << msg;
  print(ss,"   ",stream);
}
void utilities::warning(const string& fcn_name, const string& msg, ostream& stream) {
  ostringstream ss;
  ss << fcn_name << "() - " << msg;
  print(ss," * ",stream);
}
void utilities::error(const string& fcn_name, const string& msg, uint32 errnum, ostream& stream) {
  ostringstream ss;
  ss << fcn_name << "() - " << msg << " - " << strerror(errnum);
  print(ss," ! ",stream);
}

vector<string> utilities::list_files(const string& dir_path) {
  #if (POSIX_ENV)
    DIR* dir = NULL;
    struct dirent* filename = NULL;
    vector<string> files_list;

    dir = opendir(dir_path.c_str());
    filename = readdir(dir);

    while (filename != NULL) {
      string str = filename->d_name;
      if ((str != ".")&&(str != "..")) {
        files_list.push_back(filename->d_name);
      }
      filename = readdir(dir);
    }

    closedir(dir);
    return files_list;
  #elif (QT_ENV)
    UNUSED(dir_path)
    return vector<string>();
  #endif
}
string utilities::buff2bin(const void* p_source, uint32 n_bytes) {
  string msg;
  for (uint32 i=0; i<n_bytes; ++i) {
    msg += num2bin(((uint8*)p_source)[i]);
  }
  return msg;
}
string utilities::buff2hex(const void* p_source, uint32 n_bytes) {
  string msg;
  for (uint32 i=0; i<n_bytes; ++i) {
    msg += num2hex(((uint8*)p_source)[i]);
  }
  return msg;
}
string utilities::num2bin(uint8 val) {
  string msg;

  uint8 mask = 128;
  for (uint32 i=0; i<8; ++i) {
    if (val&mask) {
      msg += "1";
    }
    else {
      msg += "0";
    }  
    mask >>= 1;
  }

  return msg;
}
string utilities::num2hex(uint8 val) {
  ostringstream ss;
  ss << std::setw(2) << std::setfill('0') << std::setbase(16) << val;
  return ss.str();
}
string utilities::find_first_ipv4_containing(const string& substring) {
  string address_string;
  #if (POSIX_ENV)
    struct ifaddrs* address_list = NULL;

    // Attempt to retreive ip address list.
    if (getifaddrs(&address_list) == -1) {
      error("utilities::find_first_ipv4_containing","failed to retreive ip address list",errno);
    }
    else {
      // Iterate through ip address list.
      for (struct ifaddrs* p_addr=address_list; p_addr!=NULL; p_addr = p_addr->ifa_next) {
        // If no address found, continue.
        if (p_addr->ifa_addr == NULL) {
          continue;
        }
        // If address family is IPV4, extract ip address.
        else if (p_addr->ifa_addr->sa_family == AF_INET) {
          // Convert ip address structure to easier format.
          sockaddr_in* p_temp = (sockaddr_in*)p_addr->ifa_addr;

          // Convert ip address into string.
          string temp_string = inet_ntoa(p_temp->sin_addr);

          // Break for loop if string contains substring input.
          if (temp_string.find(substring) != string::npos) {
            address_string = temp_string;
          }
        }
      }

      // Free ip address list.
      freeifaddrs(address_list);
      address_list = NULL;
    }
  #elif (QT_ENV)
    QString q_substring = QString::fromStdString(substring);

    // Attempt to retrieve ip address list.
    QList<QHostAddress> addresses = QNetworkInterface::allAddresses();
    if(addresses.isEmpty())
    {
        error("utilities::find_first_ipv4_containing","failed to retreive ip address list",errno);
    }
    // Continue if have addresses.
    else
    {
        // Iterate through ip address list.
        foreach(QHostAddress address, addresses)
        {
            QString q_address_string = address.toString();

            // If address contains substring input, save and break loop.
            if(q_address_string.contains(q_substring))
            {
                address_string = q_address_string.toStdString();
                break;
            }
        }
    }
  #endif
  return address_string;
}
string utilities::ipv4_to_string_ip(uint32 ip) {
  uint8 a[4];
  memcpy(&a[0],&ip,4*sizeof(uint8));

  ostringstream ss;
  ss << std::setbase(10)
     << std::setfill('0')
     << std::fixed
     << std::noshowpos
     << std::setprecision(0)
     << std::setw(3) << a[0]
     << "."
     << std::setw(3) << a[1]
     << "."
     << std::setw(3) << a[2]
     << "."
     << std::setw(3) << a[3];

  return ss.str();
}
string utilities::ipv4_to_string_port(uint16 port) {
  ostringstream ss;
  ss << std::setbase(10)
     << std::setfill('0')
     << std::fixed
     << std::noshowpos
     << std::setprecision(0)
     << std::setw(4) << port;
  return ss.str();
}
string utilities::ipv4_to_string(uint32 ip, uint16 port) {
  return ipv4_to_string_ip(ip)+":"+ipv4_to_string_port(port);
}
uint32 utilities::ipv4_to_uint32(uint8 a1, uint8 a2, uint8 a3, uint8 a4) {
  uint32 A1 = (uint32)a1;
  uint32 A2 = (uint32)a2;
  uint32 A3 = (uint32)a3;
  uint32 A4 = (uint32)a4;
  return (A1<<24)|(A2<<16)|(A3<<8)|A4;
}
uint32 utilities::string_to_ipv4_ip(const string& ip) {
  // This function expects the string address to be of the form "xxx.xxx.xxx.xxx". Spaces are 
  // acceptable, and not all digits must be present in string.

  // Find first '.'.
  uint32 index1 = ip.find_first_of('.');

  // Find second '.'.
  uint32 index2 = ip.find_first_of('.',index1+1);

  // Find third '.'.
  uint32 index3 = ip.find_first_of('.',index2+1);

  // Extract sub-strings from ip string.
  string substr1 = ip.substr(0,index1);
  string substr2 = ip.substr(index1+1,index2-index1-1);
  string substr3 = ip.substr(index2+1,index3-index2-1);
  string substr4 = ip.substr(index3+1,string::npos);

  // Convert sub-strings 1-4 to uint8 for ipv4 address.
  uint8 a1 = (uint8)atoi(substr1.c_str());
  uint8 a2 = (uint8)atoi(substr2.c_str());
  uint8 a3 = (uint8)atoi(substr3.c_str());
  uint8 a4 = (uint8)atoi(substr4.c_str());

  // Return numerical address.
  return ipv4_to_uint32(a1,a2,a3,a4);
}
uint16 utilities::string_to_ipv4_port(const string& port) {
  return (uint16)atoi(port.c_str());
}
void utilities::string_to_ipv4(const string& address, uint32& ip, uint16& port) {
  // This function expects the string address to be of the form "xxx.xxx.xxx.xxx:xxxx". Spaces are 
  // acceptable, and not all digits must be present in string.

  // Find first '.'.
  uint32 index1 = address.find_first_of('.');

  // Find second '.'.
  uint32 index2 = address.find_first_of('.',index1+1);

  // Find third '.'.
  uint32 index3 = address.find_first_of('.',index2+1);

  // Find ':'.
  uint32 index4 = address.find_first_of(':',index3+1);

  // Extract sub-strings from address string.
  string substr1 = address.substr(0,index1);
  string substr2 = address.substr(index1+1,index2-index1-1);
  string substr3 = address.substr(index2+1,index3-index2-1);
  string substr4 = address.substr(index3+1,index4-index3-1);
  string substr5 = address.substr(index4+1,string::npos);

  // Convert sub-strings 1-4 to uint8 for ipv4 address.
  uint8 a1 = (uint8)atoi(substr1.c_str());
  uint8 a2 = (uint8)atoi(substr2.c_str());
  uint8 a3 = (uint8)atoi(substr3.c_str());
  uint8 a4 = (uint8)atoi(substr4.c_str());

  // Write numerical address into ip and port variables.
  ip = ipv4_to_uint32(a1,a2,a3,a4);
  port = (uint16)atoi(substr5.c_str());
}

float utilities::wrap_rad_360(float angle_rad) {
  float two_pi = 2.0f*pi;
  float revs = floor(fabs(angle_rad)/two_pi);
  float angle_sign = sign(angle_rad);
  return angle_rad-angle_sign*two_pi*revs-pi*(angle_sign-1.0f);
}
float utilities::wrap_rad_180(float angle_rad) {
  float ret = wrap_rad_360(angle_rad);
  return ret-static_cast<float>(ret > pi)*2.0f*pi;
}
float utilities::wrap_deg_360(float angle_deg) {
  float revs = floor(fabs(angle_deg)/360.0f);
  float angle_sign = sign(angle_deg);
  return angle_deg-angle_sign*360.0f*revs-180.0f*(angle_sign-1.0f);
}
float utilities::wrap_deg_180(float angle_deg) {
  float ret = wrap_deg_360(angle_deg);
  return ret-static_cast<float>(ret > 180.0f)*360.0f;
}
float utilities::tc2float(uint8 lsb, uint8 msb) {
  uint16 val = static_cast<uint16>(lsb)+(static_cast<uint16>(msb)<<8);
  return static_cast<float>((val+32768)%65536-32768);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
