////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/sc18is602b.cpp                                                          //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "fsw/hardware/sc18is602b.h"
using fsw::hardware::sc18is602b;
using utilities::i2c_port;

////////////////////////////////////////////////////////////////////////////////////////////////////

sc18is602b::sc18is602b(const i2c_port& port, uint8 address_bit) :
  object("hardware","sc18is602b"),
  mc_port(port),
  mc_i2c_base_address(0b0101000),
  mc_i2c_address(mc_i2c_base_address|address_bit) {
  // Ensure that GPIOs are disabled.
  gpio_enable(0);
  
  // Attempt to read from device buffer.
  uint8 msg[3]={};
  uint8 bytes_read = read_from_device_buffer(msg,3);
  if (bytes_read == 3) {
    message("sc18is602b","device initialized successfully");
  }
  else {
    warning("sc18is602b","device initialization failed");
  }
  
  // Enter idle mode. Device will automatically exit idle mode when it receives its next message.
  enter_idle_mode();
}
sc18is602b::~sc18is602b() {
  // Enter idle mode.
  enter_idle_mode();
  
  message("~sc18is602b","terminating device object");
}
void sc18is602b::write_to_device_buffer(const uint8* p_tx, uint32 n_bytes, CS cs) const {
  // See page 5, section 7.1.2 in SC18IS602B datasheet. Ensure that n_bytes <= 200.
  
  // Allocate buffer that is 1+200 bytes long.
  uint8 buffer[201]={};
  
  // Set first byte in buffer to cs.
  buffer[0] = cs;
  
  // Copy n_bytes bytes from p_tx to buffer[1].
  memcpy(buffer+1,p_tx,n_bytes);
  
  // Write to device.
  mc_port.write(buffer,n_bytes+1,mc_i2c_address);
}
uint32 sc18is602b::read_from_device_buffer(uint8* p_rx, uint32 n_bytes) const {
  // See page 6, section 7.1.4 in SC18IS602B datasheet. This function reads the data from the 
  // device's buffer into p_rx. Note that n_bytes <= 200.
  return mc_port.read(p_rx,n_bytes,mc_i2c_address);
}
void sc18is602b::spi_config(ORDER order, SPI_MODE spi_mode, SPI_CLK_RATE spi_clk_rate) const {
  // See page 7, section 7.1.5 in SC18IS602B datasheet.
  uint8 msg[2]={};
  msg[0] = REG_CONFIG_SPI_INTERFACE;
  msg[1] = order|spi_mode|spi_clk_rate;
  mc_port.write(msg,2,mc_i2c_address);
}
void sc18is602b::clear_interrupt() const {
  // See page 8, section 7.1.6 in SC18IS602B datasheet.
  uint8 msg[1]={};
  msg[0] = REG_CLEAR_INTERRUPT;
  mc_port.write(msg,1,mc_i2c_address);
}
void sc18is602b::enter_idle_mode() const {
  // See page 8, section 7.1.7 in SC18IS602B datasheet.
  uint8 msg[1]={};
  msg[0] = REG_IDLE_MODE;
  mc_port.write(msg,1,mc_i2c_address);
}
void sc18is602b::gpio_write(uint8 cs) const {
  // See page 8, section 7.1.8 in SC18IS602B datasheet. The value of cs should be derived from 
  // OR-operations of the defined CS0-CS3 chip select constants.
  uint8 msg[2]={};
  msg[0] = REG_GPIO_WRITE;
  msg[1] = cs;
  mc_port.write(msg,2,mc_i2c_address);
}
uint8 sc18is602b::gpio_read(uint8 cs) const {
  // See page 9, section 7.1.9 in SC18IS602B datasheet. The value of cs should be derived from 
  // OR-operations of the defined CS0-CS3 chip select constants.
  uint8 msg[2]={};
  msg[0] = REG_GPIO_READ;
  msg[1] = cs;
  mc_port.write(msg,2,mc_i2c_address);
  read_from_device_buffer(msg,1);
  return msg[0];
}
void sc18is602b::gpio_enable(uint8 cs) const {
  // See page 9, section 7.1.10 in SC18IS602B datasheet. The value of cs should be derived from 
  // OR-operations of the defined CS0-CS3 chip select constants.
  uint8 msg[2]={};
  msg[0] = REG_GPIO_ENABLE;
  msg[1] = cs;
  mc_port.write(msg,2,mc_i2c_address);
}
void sc18is602b::gpio_config(GPIO_CFG cfg0, GPIO_CFG cfg1, GPIO_CFG cfg2, GPIO_CFG cfg3) const {
  // See page 10, section 7.1.11 in SC18IS602B datasheet.
  uint8 msg[2]={};
  msg[0] = REG_GPIO_CONFIG;
  msg[1] = (cfg3<<6)|(cfg2<<4)|(cfg1<<2)|(cfg0);
  mc_port.write(msg,2,mc_i2c_address);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
