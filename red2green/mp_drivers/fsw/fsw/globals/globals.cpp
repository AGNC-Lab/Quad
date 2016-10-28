////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/globals/global_defs.cpp                                                           //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "fsw/globals/globals.h"
#include "utilities/formatting.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

void fsw::globals::preamble() {
  std::cout << std::endl;
	std::cout << formatting::separator() << std::endl;
  
  std::cout << formatting::borders() << std::endl;
  std::cout << formatting::logo() << std::endl;
  std::cout << formatting::borders() << std::endl;
  
  std::cout << formatting::separator() << std::endl;
  
  std::cout << formatting::borders() << std::endl;
  std::cout << formatting::authors() << std::endl;
  std::cout << formatting::lab() << std::endl;
  std::cout << formatting::license() << std::endl;
  std::cout << formatting::borders() << std::endl;

  std::cout << formatting::separator() << std::endl;

  std::cout << formatting::borders() << std::endl;
  std::cout << formatting::title("exec_mikipilot_fsw") << std::endl;
  std::cout << formatting::version(version::string) << std::endl;
  std::cout << formatting::borders() << std::endl;
  
  std::cout << formatting::separator() << std::endl;
  std::cout << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
