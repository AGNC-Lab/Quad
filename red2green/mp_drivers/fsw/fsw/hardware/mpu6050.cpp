////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/mpu6050.cpp                                                             //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "fsw/hardware/mpu6050.h"
using fsw::hardware::mpu6050;
using constants::deg2rad;
using constants::g0;
using utilities::i2c_port;
using utilities::tc2float;

////////////////////////////////////////////////////////////////////////////////////////////////////

const float mpu6050::sc_accl_scales[4] = {
  16384.0f,
  8192.0f,
  4096.0f,
  2048.0f
};
const float mpu6050::sc_gyro_scales[4] = {
  131.0f,
  65.5f,
  32.8f,
  16.4f
};

mpu6050::mpu6050(const i2c_port& port, ADDRESS_BIT address_bit, uint32 exec_count) : 
  module("hardware","mpu6050",exec_count),
  rx_sim_data(class_name()+"::rx_sim_data",1),
  tx_device_data(class_name()+"::tx_device_data"),
  m_module_sm(class_name(),states::imu_state::IDLE),
  m_health_sm(class_name()),
  mc_port(port),
  mc_i2c_base_address(0b1101000),
  mc_i2c_address(mc_i2c_base_address|address_bit),
  mc_temp_coeff0(36.53f),
  mc_temp_coeff1(1.0f/340.0f) {
  // Do nothing.
}

void mpu6050::step(float ts) {
  // Transmit current states.
  m_module_sm.broadcast_current_state(ts);
  m_health_sm.broadcast_current_state(ts);
  
  // Execute current state of m_module_sm.
  switch (m_module_sm.current_state().index()) {
    case states::imu_state::TERM:
      term();
      break;
    case states::imu_state::IDLE:
      idle();
      break;
    case states::imu_state::DEVICE:
      device(ts);
      break;
    case states::imu_state::SIM:
      sim(ts);
      break;
    case states::imu_state::CALIBRATE:
      calibrate(ts);
      break;
  }
}
void mpu6050::term() {
  // Process next state transition request.
  m_module_sm.process_transition_requests();
}
void mpu6050::idle() {
  // Initialize variables.
  initialize_variables();
  
  // Process next state transition request.
  if (m_module_sm.process_transition_requests()) {
    if (m_module_sm.current_state().index() == states::imu_state::DEVICE) {
      load_config_data();
      m_calib_data.load("../calib/mpu6050_calib.txt");
      id_device();
    }
  }
}
void mpu6050::device(float ts) {
  // Update IMU data from device.
  if (update(m_device_data.accl,m_device_data.gyro)) {
    // Timestamp and push data through tx_device_data.
    m_device_data.timestamp(ts);
    tx_device_data.push(m_device_data);
  }
  
  // Process next state transition request.
  m_module_sm.process_transition_requests();
}
void mpu6050::sim(float ts) {
  // Update IMU data from sim.
  if (rx_sim_data.pull(m_sim_data)) {
    // Timestamp and push data through tx_device_data.
    m_device_data << m_sim_data;
    m_device_data.timestamp(ts);
    tx_device_data.push(m_device_data);
  }
  
  // Process next state transition request.
  m_module_sm.process_transition_requests();
}
void mpu6050::calibrate(float ts) {
  if (m_i_sample == 0) {
    // Print start message.
    message("calibrate","starting calibration");
    
    // Set MPU6050 biases to zero.
    m_calib_data.accl_bias = v3x1::Zero();
    m_calib_data.gyro_bias = v3x1::Zero();
  }
  
  // Update IMU data from device.
  if (update(m_device_data.accl,m_device_data.gyro)) {
    // Timestamp and push data through tx_device_data.
    m_device_data.timestamp(ts);
    tx_device_data.push(m_device_data);
    
    // Keep running sum of measurements. The measurements are collected in the device's 
    // reference frame. Increment m_i_sample.
    m_accl_bias_sum += m_config_data.C_dev2body.transpose()*
      (m_device_data.accl-(v3x1() << 0.0f,0.0f,-g0).finished());
    m_gyro_bias_sum += m_config_data.C_dev2body.transpose()*m_device_data.gyro;
    ++m_i_sample;
  }
  
  if (m_i_sample == m_config_data.n_calib_samples) {
    // Compute biases by averaging measurements.
    m_calib_data.accl_bias = m_accl_bias_sum/static_cast<float>(m_config_data.n_calib_samples);
    m_calib_data.gyro_bias = m_gyro_bias_sum/static_cast<float>(m_config_data.n_calib_samples);
    
    // Save biases in calibration file.
    m_calib_data.save("../calib/mpu6050_calib.txt");
    
    // Zero out variables.
    m_accl_bias_sum = v3x1::Zero();
    m_gyro_bias_sum = v3x1::Zero();
    m_i_sample = 0;
    
    // Print end message.
    message("calibrate","calibration complete");
    
    // Return to previous state.
    m_module_sm.transition_to(m_module_sm.previous_state());
  }
}

void mpu6050::initialize_variables() {
  m_device_data = packets::imu_data<TOPIC_DEVICE>();
  m_sim_data = packets::imu_data<TOPIC_SIM>();
  m_inv_accl_scale = 0.0f;
  m_inv_gyro_scale = 0.0f;
  m_i_sample = 0;
  m_accl_bias_sum = v3x1::Zero();
  m_gyro_bias_sum = v3x1::Zero();
}
void mpu6050::load_config_data() {
  // Load configuration data.
  m_config_data.load("../config/mpu6050_config.txt");
  
  // Set MPU6050 clock source.
  set_clock_source();
  
  // Load MPU6050 digital low-pass filter settings.
  DLPF_CFG dlpf_cfg = static_cast<DLPF_CFG>(m_config_data.dlpf_cfg);
  set_dlpf_config(dlpf_cfg);
  
  // Load sample rate division settings.
  set_sample_rate_div(m_config_data.sample_rate_div);
  
  // Load MPU6050 accelerometer full-scale range.
  ACCL_FS accl_fs = static_cast<ACCL_FS>(m_config_data.accl_fs);
  set_accl_full_scale(accl_fs);
  
  // Load MPU6050 gyroscope full-scale range.
  GYRO_FS gyro_fs = static_cast<GYRO_FS>(m_config_data.gyro_fs);
  set_gyro_full_scale(gyro_fs);
}
void mpu6050::set_clock_source() const {
  // Write to PWR_MGMT_1 register to set clock source.
  uint8 msg[2]={};
  msg[0] = PWR_MGMT_1_REG;
  msg[1] = 0x01; // sets clock to PLL with X axis gyroscope reference.
  mc_port.write(msg,2,mc_i2c_address);
}
void mpu6050::set_dlpf_config(DLPF_CFG dlpf_cfg) const {
  // Write to CONFIG register.
  uint8 msg[2]={};
  msg[0] = CONFIG_REG;
  msg[1] = dlpf_cfg; // sets low-pass filter settings to dlpf_cfg.
  mc_port.write(msg,2,mc_i2c_address);
}
void mpu6050::set_sample_rate_div(uint8 smprt_div) const {
  // Write to SMPRT_DIV register (see pages 11-12 of MPU6050 register map datasheet).
  uint8 msg[2]={};
  msg[0] = SMPRT_DIV_REG;
  msg[1] = smprt_div; // sets sample rate to Gyroscope Output Rate.
  mc_port.write(msg,2,mc_i2c_address);
}
void mpu6050::set_accl_full_scale(ACCL_FS accl_fs) {
  // Write to ACCL_CONFIG register.
  uint8 msg[2]={};
  msg[0] = ACCL_CONFIG_REG;
  msg[1] = accl_fs<<3; // sets accelerometer full-scale range to accl_fs.
  mc_port.write(msg,2,mc_i2c_address);
  
  // Update m_inv_accl_scale.
  m_inv_accl_scale = g0/sc_accl_scales[accl_fs];
}
void mpu6050::set_gyro_full_scale(GYRO_FS gyro_fs) {
  // Write to GYRO_CONFIG register.
  uint8 msg[2]={};
  msg[0] = GYRO_CONFIG_REG;
  msg[1] = gyro_fs<<3; // sets gyroscope full-scale range to gyro_fs.
  mc_port.write(msg,2,mc_i2c_address);
  
  // Update m_inv_gyro_scale.
  m_inv_gyro_scale = 1.0f/sc_gyro_scales[gyro_fs]*deg2rad;
}
bool mpu6050::id_device() const {
  // Write to the WHO_AM_I register.
  uint8 msg[1]={};
  mc_port.read(msg,1,WHO_AM_I_REG,mc_i2c_address);
  
  // Check if device identification was successful (see page 46 of MPU6050 register map datasheet).
  if (msg[0] == 0x68) {
    message("device_id","device ID successful");
    return true;
  }
  else {
    warning("device_id","device ID failed");
    return false;
  }
}
bool mpu6050::update_accl(mv3x1& accl_data) {
  uint8 msg[6]={};
  uint32 bytes_read = mc_port.read(msg,6,ACCL_XOUT_H_REG,mc_i2c_address);
  
  if (bytes_read == 6) {
    accl_data(0) = tc2float(msg[1],msg[0])*m_inv_accl_scale-m_calib_data.accl_bias(0);
    accl_data(1) = tc2float(msg[3],msg[2])*m_inv_accl_scale-m_calib_data.accl_bias(1);
    accl_data(2) = tc2float(msg[5],msg[4])*m_inv_accl_scale-m_calib_data.accl_bias(2);
    accl_data = m_config_data.C_dev2body*accl_data;
    return true;
  }
  else {
    // Report error if device update failed.
    m_health_sm.report_error();
    warning("update_accl","accelerometer update failed");
    return false;
  }
}
bool mpu6050::update_gyro(mv3x1& gyro_data) {
  uint8 msg[6]={};
  uint32 bytes_read = mc_port.read(msg,6,GYRO_XOUT_H_REG,mc_i2c_address);
  
  if (bytes_read == 6) {
    gyro_data(0) = tc2float(msg[1],msg[0])*m_inv_gyro_scale-m_calib_data.gyro_bias(0);
    gyro_data(1) = tc2float(msg[3],msg[2])*m_inv_gyro_scale-m_calib_data.gyro_bias(1);
    gyro_data(2) = tc2float(msg[5],msg[4])*m_inv_gyro_scale-m_calib_data.gyro_bias(2);
    gyro_data = m_config_data.C_dev2body*gyro_data;
    return true;
  }
  else {
    // Report error if device update failed.
    m_health_sm.report_error();
    warning("update_gyro","gyroscope update failed");
    return false;
  }
}
bool mpu6050::update_temp(float& temp_data) {
  uint8 msg[2]={};
  uint32 bytes_read = mc_port.read(msg,2,TEMP_OUT_H_REG,mc_i2c_address);
  
  if (bytes_read == 2) {
    temp_data = tc2float(msg[1],msg[0])*mc_temp_coeff1+mc_temp_coeff0;
    return true;
  }
  else {
    // Report error if device update failed.
    m_health_sm.report_error();
    warning("update_temperature","temperature update failed");
    return false;
  }
}
bool mpu6050::update(mv3x1& accl_data, mv3x1& gyro_data) {
  uint8 msg[14]={};
  uint32 bytes_read = mc_port.read(msg,14,ACCL_XOUT_H_REG,mc_i2c_address);
  
  if (bytes_read == 14) {
    accl_data(0) = tc2float(msg[1],msg[0])*m_inv_accl_scale-m_calib_data.accl_bias(0);
    accl_data(1) = tc2float(msg[3],msg[2])*m_inv_accl_scale-m_calib_data.accl_bias(1);
    accl_data(2) = tc2float(msg[5],msg[4])*m_inv_accl_scale-m_calib_data.accl_bias(2);
    accl_data = m_config_data.C_dev2body*accl_data;
    
    gyro_data(0) = tc2float(msg[9],msg[8])*m_inv_gyro_scale-m_calib_data.gyro_bias(0);
    gyro_data(1) = tc2float(msg[11],msg[10])*m_inv_gyro_scale-m_calib_data.gyro_bias(1);
    gyro_data(2) = tc2float(msg[13],msg[12])*m_inv_gyro_scale-m_calib_data.gyro_bias(2);
    gyro_data = m_config_data.C_dev2body*gyro_data;
    
    return true;
  }
  else {
    // Report error if device update failed.
    m_health_sm.report_error();
    warning("update_all","update failed");
    return false;
  }
}
bool mpu6050::update(mv3x1& accl_data, mv3x1& gyro_data, float& temp_data) {
  uint8 msg[14]={};
  uint32 bytes_read = mc_port.read(msg,14,ACCL_XOUT_H_REG,mc_i2c_address);
  
  if (bytes_read == 14) {
    accl_data(0) = tc2float(msg[1],msg[0])*m_inv_accl_scale-m_calib_data.accl_bias(0);
    accl_data(1) = tc2float(msg[3],msg[2])*m_inv_accl_scale-m_calib_data.accl_bias(1);
    accl_data(2) = tc2float(msg[5],msg[4])*m_inv_accl_scale-m_calib_data.accl_bias(2);
    accl_data = m_config_data.C_dev2body*accl_data;
    
    temp_data = tc2float(msg[7],msg[6])*mc_temp_coeff1+mc_temp_coeff0;
    
    gyro_data(0) = tc2float(msg[9],msg[8])*m_inv_gyro_scale-m_calib_data.gyro_bias(0);
    gyro_data(1) = tc2float(msg[11],msg[10])*m_inv_gyro_scale-m_calib_data.gyro_bias(1);
    gyro_data(2) = tc2float(msg[13],msg[12])*m_inv_gyro_scale-m_calib_data.gyro_bias(2);
    gyro_data = m_config_data.C_dev2body*gyro_data;
    
    return true;
  }
  else {
    // Report error if device update failed.
    m_health_sm.report_error();
    warning("update_all","update failed");
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
