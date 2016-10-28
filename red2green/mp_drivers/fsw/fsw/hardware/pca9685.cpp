////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/pca9685.cpp                                                             //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "fsw/hardware/pca9685.h"
using fsw::hardware::overo;
using fsw::hardware::pca9685;
using std::ostringstream;
using utilities::bound;
using utilities::i2c_port;

////////////////////////////////////////////////////////////////////////////////////////////////////

pca9685::pca9685(const i2c_port& port, ADDRESS_BIT address_bit, uint32 exec_count) :
  module("hardware","pca9685",exec_count),
  rx_device_data(class_name()+"::rx_device_data",1),
  tx_sim_data(class_name()+"::tx_sim_data"),
  rx_pw_monitor_data_usec(class_name()+"::rx_pw_monitor_data_usec",1000),
  m_module_sm(class_name(),states::pwm_state::IDLE),
  mc_port(port),
  mc_i2c_base_address(0b1000000),
  mc_i2c_address(mc_i2c_base_address|address_bit),
  mc_counts(4096.0f),
  mc_nominal_ref_clk_freq(25e6f) {
  // Do nothing.
}

void pca9685::step(float ts) {
  // Transmit current state.
  m_module_sm.broadcast_current_state(ts);
  
  // Execute current state of m_module_sm.
  switch (m_module_sm.current_state().index()) {
    case states::pwm_state::TERM:
      term();
      break;
    case states::pwm_state::IDLE:
      idle();
      break;
    case states::pwm_state::DEVICE:
      device();
      break;
    case states::pwm_state::SIM:
      sim(ts);
      break;
    case states::pwm_state::CALIBRATE:
      calibrate();
      break;
  }
}
void pca9685::term() {
  // Flush receiving data buffer.
  rx_device_data.flush();
  
  // Process next state transition request.
  m_module_sm.process_transition_requests();
}
void pca9685::idle() {
  // Initialize variables.
  initialize_variables();
  
  // Flush receiving data buffer.
  rx_device_data.flush();
  
  // Zero all channels.
  zero_all();
  
  // Process next state transition request.
  if (m_module_sm.process_transition_requests()) {
    if (m_module_sm.current_state().index() == states::pwm_state::DEVICE) {
      load_config_data();
      m_calib_data.load("../calib/pca9685_calib.txt");
      set_mode1_reg();
      set_mode2_reg();
      set_pre_scale_reg(false);
      id_device();
    }
  }
}
void pca9685::device() {
  // Update PCA9685 PWMs from incoming data.
  if (rx_device_data.pull(m_device_data)) {
    // Set PCA9685 channel data.
    set_pulse_width(CH1,bound(m_config_data.min_pw_usec(0),m_device_data.ch(0),
                              m_config_data.max_pw_usec(0)));
    set_pulse_width(CH2,bound(m_config_data.min_pw_usec(1),m_device_data.ch(1),
                              m_config_data.max_pw_usec(1)));
    set_pulse_width(CH3,bound(m_config_data.min_pw_usec(2),m_device_data.ch(2),
                              m_config_data.max_pw_usec(2)));
    set_pulse_width(CH4,bound(m_config_data.min_pw_usec(3),m_device_data.ch(3),
                              m_config_data.max_pw_usec(3)));
    set_pulse_width(CH5,bound(m_config_data.min_pw_usec(4),0.0f,
                              m_config_data.max_pw_usec(4)));
    set_pulse_width(CH6,bound(m_config_data.min_pw_usec(5),0.0f,
                              m_config_data.max_pw_usec(5)));
    
    // Update PCA9685 channels.
    update(CH1,CH6);
  }
  
  // Process next state transition request.
  m_module_sm.process_transition_requests();
}
void pca9685::sim(float ts) {
  // Update simulation PWMs from incoming data.
  if (rx_device_data.pull(m_device_data)) {
    m_sim_data << m_device_data;
    m_sim_data.timestamp(ts);
    tx_sim_data.push(m_sim_data);
  }
  
  // Process next state transition request.
  m_module_sm.process_transition_requests();
}
void pca9685::calibrate() {
  if (!rx_pw_monitor_data_usec.is_connected()) {
    // Print start message.
    message("calibrate","starting calibration");
    
    // Set calibration factor to 1.0.
    m_calib_data.clock_correction_factor = 1.0f;
    set_pre_scale_reg(true);
    
    // Set all PWM channels to m_config_data.calibration_pulse_width_usec.
    set_pulse_width(CH1,m_config_data.calibration_pulse_width_usec);
    set_pulse_width(CH2,m_config_data.calibration_pulse_width_usec);
    set_pulse_width(CH3,m_config_data.calibration_pulse_width_usec);
    set_pulse_width(CH4,m_config_data.calibration_pulse_width_usec);
    set_pulse_width(CH5,m_config_data.calibration_pulse_width_usec);
    set_pulse_width(CH6,m_config_data.calibration_pulse_width_usec);
    update(CH1,CH6);
    
    // Launch GPIO_IRQ_146 pw_monitor on Overo (see PWM_IN_4_5.0V in MP AP Board R2 Schematic).
    // Connect pw_monitor to rx_pw_monitor_data_usec consumer.
    overo::launch_pw_monitor(overo::GPIO_IRQ_146) >> rx_pw_monitor_data_usec;
  }
  else if (rx_pw_monitor_data_usec.queue_length() >= m_config_data.n_calib_samples) {
    // Terminate GPIO_IRQ_146 pw_monitor on Overo. Disconnect rx_pw_monitor_data_usec.
    overo::terminate_pw_monitor(overo::GPIO_IRQ_146) /= rx_pw_monitor_data_usec;
    
    // Compute average of pulse width data arriving on rx_pw_monitor_data_usec.
    float pw_sum_usec = 0.0f;
    float pw_measurement_usec = 0.0f;
    uint32 n_pw_measurements = 0;
    
    while (rx_pw_monitor_data_usec.queue_length() > 0) {
      if (rx_pw_monitor_data_usec.pull(pw_measurement_usec)) {
        pw_sum_usec += pw_measurement_usec;
        ++n_pw_measurements;
      }
    }
    
    float pw_avg_usec = pw_sum_usec/static_cast<float>(n_pw_measurements);
    float pw_ratio = pw_avg_usec/m_config_data.calibration_pulse_width_usec;
    
    // Compute and set clock correction factor.
    m_calib_data.clock_correction_factor = 1.0f/pw_ratio;
    
    // Write to pre_scale register.
    set_pre_scale_reg(false);
    
    // Ensure that rx_pw_monitor_data_usec FIFO is empty.
    if (rx_pw_monitor_data_usec.queue_length() != 0) {
      warning("calibrate","rx_pw_monitor_data_usec FIFO is not empty");
    }
    
    // Save clock correction factor in calibration file.
    m_calib_data.save("../calib/pca9685_calib.txt");
    
    // Print end message.
    message("calibrate","calibration complete");
    
    // Return to previous state.
    m_module_sm.transition_to(m_module_sm.previous_state());
  }
}

void pca9685::initialize_variables() {
  m_device_data = packets::pwm_cmd<TOPIC_DEVICE>();
  m_sim_data = packets::pwm_cmd<TOPIC_SIM>();
  m_mode1_byte = 0;
  m_mode2_byte = 0;
  m_usec2counts = 0.0f;
  memset(&m_time_table_usec[0][0],0,16*2*sizeof(uint16));
}
void pca9685::load_config_data() {
  // Load configuration data.
  m_config_data.load("../config/pca9685_config.txt");
  
  // Load channel offsets.
  set_offset(CH1,m_config_data.channel_offset(0));
  set_offset(CH2,m_config_data.channel_offset(1));
  set_offset(CH3,m_config_data.channel_offset(2));
  set_offset(CH4,m_config_data.channel_offset(3));
  set_offset(CH5,m_config_data.channel_offset(4));
  set_offset(CH6,m_config_data.channel_offset(5));
}
void pca9685::set_mode1_reg() {
  // Initialize the mode 1 byte to disable restart mode, enable the internal clock, enable register 
  // auto-increment, enable sleep mode , and disable response to i2c subaddress 1,2,3, and All 
  // Call. See page 13 in PCA9685 datasheet.
  m_mode1_byte = 0b00000000;
  m_mode1_byte |= RST_DISABLED;
  m_mode1_byte |= INT_CLK;
  m_mode1_byte |= AI_ENABLED;
  m_mode1_byte |= AWAKE;
  m_mode1_byte |= SUB1_DNR;
  m_mode1_byte |= SUB2_DNR;
  m_mode1_byte |= SUB3_DNR;
  m_mode1_byte |= ALLCALL_DNR;

  // Write to MODE1 register.
  uint8 msg[2]={};
  msg[0] = MODE1_REG;
  msg[1] = m_mode1_byte;
  mc_port.write(msg,2,mc_i2c_address);
}
void pca9685::set_mode2_reg() {  
  // Initialize the mode 2 byte to enable inverted output, change the output on the STOP command,
  // set the outputs to totem pole, and set outputs to high-impedence state when output is 
  // disabled. See page 15 in PCA9685 datasheet.
  m_mode2_byte = 0b00000000;
  m_mode2_byte |= OUTPUT_NOT_INVERTED;
  m_mode2_byte |= CHANGE_ON_STOP;
  m_mode2_byte |= TOTEM_POLE;
  m_mode2_byte |= OD_HIGHZ;
  
  // Write to MODE2 register.
  uint8 msg[2]={};
  msg[0] = MODE2_REG;
  msg[1] = m_mode2_byte;
  mc_port.write(msg,2,mc_i2c_address);
}
void pca9685::set_device_status(SLEEP_MODE sleep_mode) { 
  // Modify mode 1 byte.
  m_mode1_byte = (m_mode1_byte&~ASLEEP)|(sleep_mode);
  
  // Write to MODE1 register.
  uint8 msg[2]={};
  msg[0] = MODE1_REG;
  msg[1] = m_mode1_byte;
  mc_port.write(msg,2,mc_i2c_address);
}
void pca9685::set_pre_scale_reg(bool calibrate) {
  // The device must be put in sleep mode in order to write to the PRE_SCALE register successfully.
  // Note that the user must call set_device_status(AWAKE) after this function in order to resume
  // operation.
  set_device_status(ASLEEP);
  
  // Compute pre_scale.
  float ref_clk_freq = mc_nominal_ref_clk_freq*m_calib_data.clock_correction_factor;
  float pre_scale;
  if (calibrate) {
    pre_scale = roundf(ref_clk_freq/(mc_counts*m_config_data.calibration_update_rate))-1.0f;
  }
  else {
    pre_scale = roundf(ref_clk_freq/(mc_counts*m_config_data.update_rate))-1.0f;
  }
  
  // See page 24 in PCA9685 datasheet. Note, pre_scale must be greater than or equal to 3 according
  // to the datasheet.
  if (pre_scale < 3.0f) {
    ostringstream msg;
    msg << "attempted to set pre_scale to "
        << static_cast<uint32>(pre_scale)
        << " , setting pre_scale to 3";
    warning("set_pre_scale_reg",msg.str());
    pre_scale = 3.0f;
  }
  
  // Write to PRE_SCALE register.
  uint8 msg[2]={};
  msg[0] = PRE_SCALE_REG;
  msg[1] = static_cast<uint8>(pre_scale);
  mc_port.write(msg,2,mc_i2c_address);
  
  // Compute new conversion factor for converting time in micro-seconds to counts (out of 4096).
  m_usec2counts = 1e-6f*ref_clk_freq/(pre_scale+1.0f);
  
  // Wake device back up.
  set_device_status(AWAKE);
}
void pca9685::set_offset(CH ch, float offset_usec) {
  // This function assumes that 0.0 < offset_usec+pwm_usec < period of pulse.
  m_time_table_usec[ch][0] = static_cast<uint16>(offset_usec*m_usec2counts);
}
void pca9685::set_pulse_width(CH ch, float pulse_width_usec) {
  // This function assumes that 0.0 < offset_usec+pwm_usec < period of pulse.
  m_time_table_usec[ch][1] = m_time_table_usec[ch][0]
    +static_cast<uint16>(pulse_width_usec*m_usec2counts);
}
void pca9685::zero_all() const {
  // This function zeros all of the outputs by writing to the ALL_LED registers. Note that the data
  // in m_times_table is not modified in this function.
  uint8 msg[2]={};
  msg[0] = ALL_LED_OFF_H_REG;
  msg[1] = 0b00010000;
  mc_port.write(msg,2,mc_i2c_address);
}
void pca9685::id_device() const {
  // Read MODE1 and MODE2 registers. The read is performed individually in the event that the 
  // register auto increment flag was not set correctly.
  uint8 msg[2]={};
  mc_port.read(msg,1,MODE1_REG,mc_i2c_address);
  mc_port.read(msg+1,1,MODE2_REG,mc_i2c_address);
  
  // Check that MODE1 and MODE2 registers are synced with m_mode1_byte and m_mode2_byte.
  if ((msg[0] == m_mode1_byte)&&(msg[1] == m_mode2_byte)) {
    message("id_device","device ID successful");
  }
  else {
    warning("id_device","device ID failed");
  }
}
void pca9685::update(CH ch) const {
  uint8 msg[5]={};
  msg[0] = LED0_ON_L_REG+4*ch;
  mc_port.write(msg,5,mc_i2c_address);
}
void pca9685::update(CH low_ch, CH high_ch) const {
  // This function assumes that high_ch >= low_ch.
  uint8 msg[1+4*16]={};
  uint32 channel_bytes = 4*(high_ch-low_ch+1);
  
  msg[0] = LED0_ON_L_REG+4*low_ch;
  memcpy(msg+1,m_time_table_usec[low_ch],channel_bytes);
  mc_port.write(msg,1+channel_bytes,mc_i2c_address);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
