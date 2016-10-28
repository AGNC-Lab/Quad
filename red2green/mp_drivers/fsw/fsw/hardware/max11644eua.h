////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/max11644eua.h                                                           //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FSW_HARDWARE_MAX11644EUA_H
#define FSW_HARDWARE_MAX11644EUA_H

#include "fsw/hardware/local_defs.h"
#include "utilities/i2c_port.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace fsw {
  namespace hardware {
    class max11644eua {
      public:
        enum SEL{SEL0=0b00010000,SEL1=0b00100000,SEL2=0b01000000};
        enum CLOCK{INTERNAL,EXTERNAL=0b00001000};
        enum TRANSFER_FUNC{UNIPOLAR,BIPOLAR=0b00000100};
        enum RESET{RESET_CONFIG,NO_ACTION=0b00000010};
        enum SCAN{SCAN00,SCAN01=0b00100000,SCAN11=0b01100000};
        enum CHANNEL_SELECTION{AIN0,AIN1=0b00000010};
        enum POLARITY{DIFFERENTIAL,SINGLE_ENDED=0b00000001};
        
      private:
        const utilities::i2c_port& m_port;
        const uint8 m_i2c_address;
        float m_counts_to_volts;
        
      public:
        max11644eua(const utilities::i2c_port& port);
        ~max11644eua();
        void set_reference_voltage(float ref_voltage);
        void setup(uint8 sel, CLOCK clock, TRANSFER_FUNC transfer_func, RESET reset) const;
        void config(SCAN scan, CHANNEL_SELECTION channel_selection, POLARITY polarity) const;
        float read_voltage(TRANSFER_FUNC transfer_func) const;
    };
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
