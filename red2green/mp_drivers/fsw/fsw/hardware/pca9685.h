////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/pca9685.h                                                               //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FSW_HARDWARE_PCA9685_H
#define FSW_HARDWARE_PCA9685_H

#include "fsw/globals/module.h"
#include "fsw/hardware/globals.h"
#include "fsw/hardware/overo.h"
#include "params/all.h"
#include "states/all.h"
#include "packets/all.h"
#include "utilities/consumer.h"
#include "utilities/producer.h"
#include "utilities/i2c_port.h"
#include "utilities/state_machine.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace fsw {
  namespace hardware {
    class pca9685 : public globals::module {
      public:
        enum ADDRESS_BIT {
          DEFAULT_ADDRESS=0b0000000,
          A0=0b0000001,
          A1=0b0000010,
          A2=0b0000100,
          A3=0b0001000,
          A4=0b0010000,
          A5=0b0100000
        };
        enum CH {
          CH1,
          CH2,
          CH3,
          CH4,
          CH5,
          CH6,
          CH7,
          CH8,
          CH9,
          CH10,
          CH11,
          CH12,
          CH13,
          CH14,
          CH15,
          CH16
        };
        enum RESTART_MODE {
          RST_DISABLED,
          RST_ENABLED=0b10000000
        };
        enum EXT_CLK_MODE {
          INT_CLK,
          EXT_CLK=0b01000000
        };
        enum AUTO_INC_MODE {
          AI_DISABLED,
          AI_ENABLED=0b00100000
        };
        enum SLEEP_MODE {
          AWAKE,ASLEEP=0b00010000
        };
        enum SUB1_MODE {
          SUB1_DNR,
          SUB1_RES=0b00001000
        };
        enum SUB2_MODE {
          SUB2_DNR,
          SUB2_RES=0b00000100
        };
        enum SUB3_MODE {
          SUB3_DNR,
          SUB3_RES=0b00000010
        };
        enum ALLCALL_MODE {
          ALLCALL_DNR,
          ALLCALL_RES=0b00000001
        };
        enum INVRT_MODE {
          OUTPUT_NOT_INVERTED,
          OUTPUT_INVERTED=0b00010000
        };
        enum OCH_MODE {
          CHANGE_ON_STOP,
          CHANGE_ON_ACK=0b00001000
        };
        enum OUTDRV_MODE {
          OPEN_DRAIN,
          TOTEM_POLE=0b00000100
        };
        enum OUTNE_MODE {
          OD_LOW,
          OD_HIGH,
          OD_HIGHZ
        };
        enum REG {
          MODE1_REG,
          MODE2_REG,
          SUBADR1,
          SUBADR2,
          SUBADR3,
          ALL_CALL_ADR_REG,
          LED0_ON_L_REG,
          LED0_ON_H_REG,
          LED0_OFF_L_REG,
          LED0_OFF_H_REG,
          ALL_LED_ON_L_REG=0xFA,
          ALL_LED_ON_H_REG,
          ALL_LED_OFF_L_REG,
          ALL_LED_OFF_H_REG,
          PRE_SCALE_REG,
          TEST_MODE_REG
        };
        
      public:
        utilities::consumer< packets::pwm_cmd<TOPIC_DEVICE> > rx_device_data;
        utilities::producer< packets::pwm_cmd<TOPIC_SIM> > tx_sim_data;
      private:
        utilities::consumer<float> rx_pw_monitor_data_usec;
        
      public:
        utilities::state_machine< states::pwm_state::state<> > m_module_sm;
        
      private:
        const utilities::i2c_port& mc_port;
        const uint8 mc_i2c_base_address;
        const uint8 mc_i2c_address;
        const float mc_counts;
        const float mc_nominal_ref_clk_freq;
        packets::pwm_cmd<TOPIC_DEVICE> m_device_data;
        packets::pwm_cmd<TOPIC_SIM> m_sim_data;
        params::pca9685_config<TOPIC_ONBOARD> m_config_data;
        params::pca9685_calib<TOPIC_ONBOARD> m_calib_data;
        uint8 m_mode1_byte;
        uint8 m_mode2_byte;
        float m_usec2counts;
        float m_ref_clk_freq_factor;
        uint16 m_time_table_usec[16][2];
        
      public:
        pca9685(const utilities::i2c_port& port, ADDRESS_BIT address_bit, uint32 exec_count);
      private:
        void step(float ts);
        void term();
        void idle();
        void device();
        void sim(float ts);
        void calibrate();
      private:
        void initialize_variables();
        void load_config_data();
        void set_mode1_reg();
        void set_mode2_reg();
        void set_device_status(SLEEP_MODE sleep_mode);
        void set_pre_scale_reg(bool calibration_mode);
        void set_offset(CH ch, float offset_usec);
        void set_pulse_width(CH ch, float pulse_width_usec);
        void zero_all() const;
        void id_device() const;
        void update(CH ch) const;
        void update(CH low_ch, CH high_ch) const;
    };
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
