////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/max11644eua.cpp                                                         //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "fsw/hardware/max11644eua.h"
using fsw::hardware::max11644eua;
using utilities::i2c_port;
using utilities::message;
using utilities::warning;

////////////////////////////////////////////////////////////////////////////////////////////////////

max11644eua::max11644eua(const i2c_port& port) : m_port(port),m_i2c_address(0b0110110) {
  // Set reference voltage.
  set_reference_voltage(4.096f);
  
  message("fsw::hardware::max11644eua::max11644eua","device initialized successfully");
}
max11644eua::~max11644eua() {
  message("fsw::hardware::max11644eua::~max11644eua","terminating device object");
}
void max11644eua::set_reference_voltage(float ref_voltage) {
  m_counts_to_volts = ref_voltage/4095.0f;
}
void max11644eua::setup(uint8 sel, CLOCK clock, TRANSFER_FUNC transfer_func, RESET reset) const {
  // See page 14, table 1 in MAX11644EUA datasheet. The value of sel should be derived from 
  // OR-operations of the defined SEL constants. See page 18, table 6 for more details on sel.
  uint8 msg[1]={};
  msg[0] = 0b10000000|sel|clock|transfer_func|reset;
  m_port.write(msg,1,m_i2c_address);
}
void max11644eua::config(SCAN scan, CHANNEL_SELECTION channel_selection, POLARITY polarity) const {
  // See page 15, table 2 in MAX11644EUA datasheet.
  uint8 msg[1]={};
  msg[0] = scan|channel_selection|polarity;
  m_port.write(msg,1,m_i2c_address);
}
float max11644eua::read_voltage(TRANSFER_FUNC transfer_func) const {
  // Read values out of device FIFO (12 bits of data).
  uint8 msg[2]={};
  uint32 bytes_read = m_port.read(msg,2,m_i2c_address);
  
  // Convert read data into counts.
  if (bytes_read == 2) {
    uint16 counts = 0;
    switch (transfer_func) {
      case UNIPOLAR:
        // Copy counts from read buffer to uint16.
        counts = (((uint16)msg[0])<<8)|((uint16)msg[1]);
        
        // Mask upper 4 bits since this is a 12-bit ADC. counts is now a value between 0-4096.
        counts &= 0x0FFF;
        
        break;
      case BIPOLAR:
        // WIP.
        break;
    }
    
    // Convert counts to engineering units.
    float voltage = ((float)counts)*m_counts_to_volts;
    return voltage;
  }
  else {
    warning("fsw::hardware::max11644eua::read_voltage","failed to read voltage");
    return 0.0f;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
