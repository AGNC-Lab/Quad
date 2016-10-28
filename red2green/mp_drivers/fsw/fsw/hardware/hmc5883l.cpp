////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/hmc5883l.cpp                                                            //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "fsw/hardware/hmc5883l.h"
using fsw::hardware::hmc5883l;
using utilities::i2c_port;
using utilities::tc2float;

////////////////////////////////////////////////////////////////////////////////////////////////////

const float hmc5883l::sc_scales[8] = {
  1370.0f,
  1090.0f,
  820.0f,
  660.0f,
  440.0f,
  390.0f,
  330.0f,
  230.0f
};

hmc5883l::hmc5883l(const i2c_port& port, uint32 exec_count) :
  module("hardware","hmc5883l",exec_count),
  rx_mocap_data(class_name()+"::rx_mocap_data",1),
  rx_sim_data(class_name()+"::rx_sim_data",1),
  tx_device_data(class_name()+"::tx_device_data"),
  m_module_sm(class_name(),states::mag_state::IDLE),
  m_health_sm(class_name()),
  mc_port(port),
  mc_i2c_address(0b0011110) {
  // Do nothing.
}

void hmc5883l::step(float ts) {
  // Transmit current state.
  m_module_sm.broadcast_current_state(ts);
  m_health_sm.broadcast_current_state(ts);
  
  // Execute current state of m_module_sm.
  switch (m_module_sm.current_state().index()) {
    case states::mag_state::TERM:
      term();
      break;
    case states::mag_state::IDLE:
      idle();
      break;
    case states::mag_state::DEVICE:
      device(ts);
      break;
    case states::mag_state::MOCAP:
      mocap(ts);
      break;
    case states::mag_state::SIM:
      sim(ts);
      break;
    case states::mag_state::CALIBRATE:
      calibrate(ts);
      break;
  }
}
void hmc5883l::term() {
  // Process next state transition request.
  m_module_sm.process_transition_requests();
}
void hmc5883l::idle() {
  // Initialize variables.
  initialize_variables();
  
  // Flush consumers.
  rx_mocap_data.flush();
  rx_sim_data.flush();
  
  // Process next state transition request.
  if (m_module_sm.process_transition_requests()) {
    if (m_module_sm.current_state().index() == states::mag_state::DEVICE) {
      load_config_data();
      m_calib_data.load("../calib/hmc5883l_calib.txt");
      id_device();
    }
  }
}
void hmc5883l::device(float ts) {
  if (rx_mocap_data.new_data_available()) {
    m_module_sm.transition_to(states::mag_state::MOCAP);
    mocap(ts);
  }
  else {
    // Update magnetometer data from device.
    if (update(m_device_data.mag_vec)) {
      // Timestamp and push data through tx_device_data.
      m_device_data.timestamp(ts);
      tx_device_data.push(m_device_data);
    }
    
    // Process next state transition request.
    m_module_sm.process_transition_requests();
  }
}
void hmc5883l::mocap(float ts) {
  // Update magnetometer data from mocap.
  if (rx_mocap_data.pull(m_mocap_data)) {
    // Timestamp and push data through tx_device_data.
    m_device_data << m_mocap_data;
    m_device_data.timestamp(ts);
    tx_device_data.push(m_device_data);
  }
  else {
    /*DEBUG*/ // Add logic to switch to DEVICE after time elapsed.
  }
  
  // Process next state transition request.
  m_module_sm.process_transition_requests();
}
void hmc5883l::sim(float ts) {
  // Update magnetometer data from sim.
  if (rx_sim_data.pull(m_sim_data)) {
    // Timestamp and push data through tx_device_data.
    m_device_data << m_sim_data;
    m_device_data.timestamp(ts);
    tx_device_data.push(m_device_data);
  }
  
  // Process next state transition request.
  m_module_sm.process_transition_requests();
}
void hmc5883l::calibrate(float ts) {
  if (m_i_sample == 0) {
    // Print start message.
    message("calibrate","starting calibration");
    
    // Set HMC5883L bias to zero.
    m_calib_data.mag_bias = v3x1::Zero();
    
    // Dynamically allocate measurement buffer. Every element will be overwritten, so there 
    // is no need to zero out the matrix.
    mp_mag_samples = new mDxD(3,m_config_data.n_calib_samples);
    (*mp_mag_samples) = mDxD::Zero(3,m_config_data.n_calib_samples);
  }
  
  // Update magnetometer data from device.
  if (update(m_device_data.mag_vec)) {
    // Timestamp and push data through tx_device_data.
    m_device_data.timestamp(ts);
    tx_device_data.push(m_device_data);
    
    // Store magnetometer measurements in measurement buffer. The measurements are 
    // collected in the device's reference frame. Increment m_i_sample.
    mp_mag_samples->col(m_i_sample) = m_config_data.C_dev2body.transpose()*m_device_data.mag_vec;
    ++m_i_sample;
  }
  
  if (m_i_sample == m_config_data.n_calib_samples) {
    // Initialize H matrix for least-squares estimation.
    mDxD H(m_config_data.n_calib_samples,4);
    H.col(0) = mDxD::Constant(m_config_data.n_calib_samples,1,1.0f);
    H.col(1) = 2.0f*mp_mag_samples->row(0).transpose();
    H.col(2) = 2.0f*mp_mag_samples->row(1).transpose();
    H.col(3) = 2.0f*mp_mag_samples->row(2).transpose();
    
    // Initialize y vector for least-squares estimation.
    mDxD y(m_config_data.n_calib_samples,1);
    y = mDxD::Constant(m_config_data.n_calib_samples,1,powf(mp_mag_samples->norm(),2.0f));
    
    // Compute biases by fitting sphere to measurements through least-squares.
    v4x1 x_hat = (H.transpose()*H).inverse()*H.transpose()*y;
    m_calib_data.mag_bias = x_hat.segment<3>(1);
    
    // Save biases in calibration file.
    m_calib_data.save("../calib/hmc5883l_calib.txt");
    
    // Zero out variables.
    m_i_sample = 0;
    delete mp_mag_samples;
    mp_mag_samples = NULL;
    
    // Print end message.
    message("calibrate","calibration complete");
    
    // Return to previous state.
    m_module_sm.transition_to(m_module_sm.previous_state());
  }
}

void hmc5883l::initialize_variables() {
  m_device_data = packets::mag_data<TOPIC_DEVICE>();
  m_sim_data = packets::mag_data<TOPIC_SIM>();
  m_mocap_data = packets::mag_data<TOPIC_MOCAP>();
  m_config_data = params::hmc5883l_config<TOPIC_ONBOARD>();
  m_calib_data = params::hmc5883l_calib<TOPIC_ONBOARD>();
  m_inv_scale = 0.0f;
  m_i_sample = 0;
  mp_mag_samples = NULL;
}
void hmc5883l::load_config_data() {
  // Load configuration data.
  m_config_data.load("../config/hmc5883l_config.txt");
  
  // Load HMC5883L operation mode.
  OP_MODE op_mode = static_cast<OP_MODE>(m_config_data.op_mode);
  set_op_mode(op_mode);
  
  // Load HMC5883L sampling settings.
  AVG_SAMP avg_samp = static_cast<AVG_SAMP>(m_config_data.avg_samp);
  RATE rate = static_cast<RATE>(m_config_data.rate);
  MEAS_MODE meas_mode = static_cast<MEAS_MODE>(m_config_data.meas_mode);
  set_config(avg_samp,rate,meas_mode);
  
  // Load HMC5883L full-scale range.
  FS_MAG fs_mag = static_cast<FS_MAG>(m_config_data.fs_mag);
  set_full_scale(fs_mag);
}
void hmc5883l::set_op_mode(OP_MODE op_mode) const {
  // Write to MODE register.
  uint8 msg[2]={};
  msg[0] = MODE_REG;
  msg[1] = op_mode;
  mc_port.write(msg,2,mc_i2c_address);
}
void hmc5883l::set_config(AVG_SAMP avg_samp, RATE rate, MEAS_MODE meas_mode) const {
  // Write to CONFIG_A register.
  uint8 msg[2]={};
  msg[0] = CONFIG_A_REG;
  msg[1] = (avg_samp<<5)|(rate<<2)|(meas_mode);
  mc_port.write(msg,2,mc_i2c_address);
}
void hmc5883l::set_full_scale(FS_MAG fs_mag) {
  // Write to CONFIG_B register.
  uint8 msg[2]={};
  msg[0] = CONFIG_B_REG;
  msg[1] = (fs_mag<<5);
  mc_port.write(msg,2,mc_i2c_address);
  m_inv_scale = 1.0f/sc_scales[fs_mag];
}
bool hmc5883l::id_device() const {
  // Write to ID_A, ID_B, and ID_C registers.
  uint8 msg[3]={};
  mc_port.read(msg,1,ID_A_REG,mc_i2c_address);
  mc_port.read(msg+1,1,ID_B_REG,mc_i2c_address);
  mc_port.read(msg+2,1,ID_C_REG,mc_i2c_address);
  
  // Check if device identification was successful (see page 17 of HMC5883L datasheet).
  if ((msg[0] == 0x48)&&(msg[1] == 0x34)&&(msg[2] == 0x33)) {
    message("device_id","device ID successful");
    return true;
  }
  else {
    warning("device_id","device ID failed");
    return false;
  }
}
bool hmc5883l::update(mv3x1& mag_vec) {
  uint8 msg[6]={};
  uint32 bytes_read = mc_port.read(msg,6,DATA_X_MSB_REG,mc_i2c_address);
  
  if (bytes_read == 6) {
    mag_vec(0) = tc2float(msg[1],msg[0])*m_inv_scale-m_calib_data.mag_bias(0);
    mag_vec(1) = tc2float(msg[5],msg[4])*m_inv_scale-m_calib_data.mag_bias(1);
    mag_vec(2) = tc2float(msg[3],msg[2])*m_inv_scale-m_calib_data.mag_bias(2);
    mag_vec = m_config_data.C_dev2body*mag_vec;
    return true;
  }
  else {
    // Report error if device update failed.
    m_health_sm.report_error();
    warning("update","failed to update device");
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
