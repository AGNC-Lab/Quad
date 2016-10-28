////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/formatting.cpp                                                              //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/formatting.h"
using std::ostringstream;
using std::string;

////////////////////////////////////////////////////////////////////////////////////////////////////

string formatting::logo() {
  const char* ascii_logo[] = {
    "      @@@@@    @@@@@   @@@   @@@         @@@  +@@@@@@@@@    @@@  @@@@               @@@ ",
    "     +@@@@@   @@@@@@  @@@   @@@         @@@   @@@@@@@@@@+  @@@  +@@@               @@@  ",
    "     @@@+@@  @@@@@@        +@@@  @@@@        +@@@+  @@@@        @@@+   +@@@@@@   @@@@@@@",
    "    +@@@+@@ @@@:@@@  @@@   @@@ @@@@    @@@   @@@@@@@@@@   @@@  +@@@  +@@@+ @@@@  @@@@@@ ",
    "    @@@ +@@@@@ @@@  +@@+  +@@@@@@@    +@@+  @@@@@@@@@+   +@@+  @@@+ +@@@   @@@@  @@@+   ",
    "   +@@+ @@@@@ +@@+  @@@   @@@@ @@@;   @@@   @@@+         @@@  +@@@  @@@+  +@@@  +@@@    ",
    "   @@@  @@@@  @@@  +@@+  @@@@  @@@@  +@@+  @@@@         +@@+  @@@+  @@@@@@@@#   @@@@@+  ",
    "  @@@   @@@  @@@   @@@   @@@   +@@@  @@@  +@@@+         @@@  @@@@    +@@@@+     +@@@+   ",
    "                                                                                        ",
    " @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  ",
    "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@   ",
  };

  string str;
  for (uint32 k=0; k<11; ++k) {
    str += "//    "+string(ascii_logo[k])+"    //";
    if (k < 10) {
      str += "\n";
    }
  }

  return str;
}
string formatting::separator() {
  string str;
  for (uint32 i=0; i<100; ++i) {
    str += "/";
  }
  return str;
}
string formatting::borders(const string& substr) {
  string str;

  str += "// ";
  str += substr;

  uint32 n_spaces = 98-str.size();
  for (uint32 i=0; i<n_spaces; ++i) {
    str += " ";
  }

  str += "//";

  return str;
}
string formatting::title(const string& title) {
  return borders("TITLE:    "+title);
}
string formatting::date() {
  #if (POSIX_ENV)
    time_t raw_time;
    struct tm* time_info;
    char time_str_buff[80];
    time(&raw_time);
    time_info = localtime(&raw_time);
    strftime(time_str_buff,80,"%c",time_info);
    string time_str = time_str_buff;

    return borders("DATE:     "+time_str);
  #elif (QT_ENV)
    return string("");/*DEBUG*/
  #endif
}
string formatting::authors() {
  return borders("AUTHORS:  Miki Szmuk");
}
string formatting::lab() {
  return borders("LAB:      Autonomous GN&C Lab");
}
string formatting::license() {
  return borders("LICENSE:  Copyright 2016, All Rights Reserved");
}
string formatting::version(const string& version) {
  return borders("VERSION:  "+version);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
