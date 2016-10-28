////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/sc18is602b.h                                                            //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FSW_HARDWARE_SC18IS602B_H
#define FSW_HARDWARE_SC18IS602B_H

#include "fsw/hardware/globals.h"
#include "fsw/globals/object.h"
#include "utilities/i2c_port.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace fsw {
  namespace hardware {
    class sc18is602b : public globals::object {
      public:
        enum ADDRESS_BIT {
          DEFAULT_ADDRESS=0b0000000,
          A0=0b0000001,
          A1=0b0000010,
          A2=0b0000100
        };
        enum CS {
          CS0=0b00000001,
          CS1=0b00000010,
          CS2=0b00000100,
          CS3=0b00001000
        };
        enum ORDER {
          MSB_FIRST,
          LSB_FIRST=0b00100000
        };
        enum SPI_MODE {
          CPOL0_CPHA0,
          CPOL0_CPHA1=0b00000100,
          CPOL1_CPHA0=0b00001000,
          CPOL1_CPHA1=0b00001100
        };
        enum SPI_CLK_RATE {
          CLKRATE_1843KHZ,
          CLKRATE_461KHZ,
          CLKRATE_115KHZ,
          CLKRATE_58KHZ
        };
        enum GPIO_CFG {
          QUASI_BIDIRECTIONAL,
          PUSH_PULL,
          INPUT_ONLY,
          OPEN_DRAIN
        };
        enum REG {
          REG_CONFIG_SPI_INTERFACE=0xF0,
          REG_CLEAR_INTERRUPT,
          REG_IDLE_MODE,
          REG_GPIO_WRITE=0xF4,
          REG_GPIO_READ,
          REG_GPIO_ENABLE,
          REG_GPIO_CONFIG
        };
        
      private:
        const utilities::i2c_port& mc_port;
        const uint8 mc_i2c_base_address;
        const uint8 mc_i2c_address;
        
      public:
        sc18is602b(const utilities::i2c_port& port, uint8 address_bit);
        ~sc18is602b();
        void write_to_device_buffer(const uint8* p_tx, uint32 n_bytes, CS cs) const;
        uint32 read_from_device_buffer(uint8* p_rx, uint32 n_bytes) const;
        void spi_config(ORDER order, SPI_MODE spi_mode, SPI_CLK_RATE spi_clk_rate) const;
        void clear_interrupt() const;
        void enter_idle_mode() const;
        void gpio_write(uint8 cs) const;
        uint8 gpio_read(uint8 cs) const;
        void gpio_enable(uint8 cs) const;
        void gpio_config(GPIO_CFG cfg0, GPIO_CFG cfg1, GPIO_CFG cfg2, GPIO_CFG cfg3) const;
    };
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
