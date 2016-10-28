////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/exec/mikipilot_fsw.cpp                                                           //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "fsw/globals/globals.h"
#include "fsw/hardware/mp_ap_board.h"
#include "fsw/hardware/overo.h"
#include "packets/all.h"
//#include "utilities/consumer.h"
//#include "utilities/globals.h"
//#include "utilities/producer.h"
//#include "utilities/timer.h"
using std::string;

////////////////////////////////////////////////////////////////////////////////////////////////////

int main() {
  // Initialize hardware and GNC objects.
  // Initialize hardware and GNC objects.
  comm::meta::set_local_node_id(comm::meta::NODE_001);

  fsw::hardware::overo overo;
  fsw::hardware::mp_ap_board mp_ap_board(overo,1,1,1);

  // Can call this if the MP AP board powers up with the red (D2) LED lit (that means that 
  // the board is in manual inputs mode, and that it will ignore what the autopilot is commanding
  // on the PWM channels). This happens occasionally, but is only an issue during power up. 
  // Calling the function below will put the board in autopilot command mode, and you'll see 
  // D2 turn off, and D1 (the green LED) turn on.
  mp_ap_board.set_pwm_mux_to_ap();

  return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
