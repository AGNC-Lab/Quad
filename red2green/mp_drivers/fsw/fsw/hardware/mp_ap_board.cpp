////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/mp_ap_board.cpp                                                         //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "fsw/hardware/mp_ap_board.h"
using fsw::hardware::hmc5883l;
using fsw::hardware::mp_ap_board;
using fsw::hardware::mpu6050;
using fsw::hardware::overo;
using fsw::hardware::pca9685;

////////////////////////////////////////////////////////////////////////////////////////////////////

mp_ap_board::mp_ap_board(const overo& device,
                         uint32 imu_exec_count,
                         uint32 mag_exec_count,
                         uint32 pwm_exec_count) : 
  object("hardware","mp_ap_board"),
  mc_overo(device),
  m_imu(device.i2c_port(),mpu6050::DEFAULT_ADDRESS,imu_exec_count),
  m_mag(device.i2c_port(),mag_exec_count),
  m_pwm(device.i2c_port(),pca9685::DEFAULT_ADDRESS,pwm_exec_count) {
  // Load configuration file and configure sensor mux/switch states.
  load_config_data();
}
mpu6050& mp_ap_board::imu() {
  return m_imu;
}
hmc5883l& mp_ap_board::mag() {
  return m_mag;
}
pca9685& mp_ap_board::pwm() {
  return m_pwm;
}
void mp_ap_board::load_config_data() {
  // Load configuration data.
  m_config_data.load("../config/mp_ap_board_config.txt");
  
  // Read data.
  IMU_MUX_STATE imu_mux_state = static_cast<IMU_MUX_STATE>(m_config_data.imu_mux_state);
  MAG_SWITCH_STATE mag_switch_state = static_cast<MAG_SWITCH_STATE>(m_config_data.mag_switch_state);
  ALT_SWITCH_STATE alt_switch_state = static_cast<ALT_SWITCH_STATE>(m_config_data.alt_switch_state);
  
  // Configure hardware.
  mpu6050_multiplexer(imu_mux_state);
  hmc5883l_switch(mag_switch_state);
  mpl3115a2_switch(alt_switch_state);
}
void mp_ap_board::set_pwm_mux_to_ap() const {
  // This function is intended to toggle the PWM mux on the MikiPilot AP Board (R2) to autopilot
  // mode (green LED). Once done, the channel is turned back into an input to avoid contention 
  // if something is driving this GPIO. It is assumed that JP2 is populated (see page 5 of 
  // MikiPilot AP Board (R2) Schematics).
  mc_overo.gpio_set_direction(overo::GPIO_147,overo::OUT);
  mc_overo.gpio_set_value(overo::GPIO_147,overo::HIGH);
  mc_overo.gpio_set_value(overo::GPIO_147,overo::LOW);
  mc_overo.gpio_set_direction(overo::GPIO_147,overo::IN);
}
void mp_ap_board::mpu6050_multiplexer(IMU_MUX_STATE state) const {
  // This function changes the state of the 4-channel SPI/I2C multiplexer (U11 on MikiPilot AP 
  // Board (R2)) by changing the value of GPIO 145 (see U8 in MikiPilot AP Board (R2) Schematics). 
  // Note that GPIO 145 is always configured as an output.
  switch (state) {
    case SELECT_I2C:
      mc_overo.gpio_set_value(overo::GPIO_145,overo::LOW);
      break;
    case SELECT_SPI:
      mc_overo.gpio_set_value(overo::GPIO_145,overo::HIGH);
      break;
  }
}
void mp_ap_board::hmc5883l_switch(MAG_SWITCH_STATE state) const {
  // This function changes the state of I2C switch (U13 on MikiPilot AP Board (R2)) by changing 
  // the value of GPIO 168 (see U8 in MikiPilot AP Board (R2) Schematics). Note that GPIO 168 is 
  // always configured as an output.
  switch (state) {
    case SELECT_OFFBOARD_MAG:
      mc_overo.gpio_set_value(overo::GPIO_168,overo::LOW);
      break;
    case SELECT_ONBOARD_MAG:
      mc_overo.gpio_set_value(overo::GPIO_168,overo::HIGH);
      break;
  }
}
void mp_ap_board::mpl3115a2_switch(ALT_SWITCH_STATE state) const {
  // This function changes the state of I2C switch (U14 on MikiPilot AP Board (R2)) by changing 
  // the value of GPIO 144 (see U8 in MikiPilot AP Board (R2) Schematics). Note that GPIO 144 is 
  // always configured as an output.
  switch (state) {
    case SELECT_OFFBOARD_ALT:
      mc_overo.gpio_set_value(overo::GPIO_144,overo::LOW);
      break;
    case SELECT_ONBOARD_ALT:
      mc_overo.gpio_set_value(overo::GPIO_144,overo::HIGH);
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
