////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/hmc5883l.h                                                              //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FSW_HARDWARE_HMC5883L_H
#define FSW_HARDWARE_HMC5883L_H

#include "fsw/globals/module.h"
#include "fsw/globals/health_sm.h"
#include "fsw/hardware/globals.h"
#include "packets/all.h"
#include "params/all.h"
#include "states/all.h"
#include "utilities/consumer.h"
#include "utilities/i2c_port.h"
#include "utilities/producer.h"
#include "utilities/state_machine.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace fsw {
  namespace hardware {
    class hmc5883l : public globals::module {
      public:
        enum AVG_SAMP {
          AVG1,
          AVG2,
          AVG4,
          AVG8
        };
        enum RATE {
          RT0075,
          RT0150,
          RT0300,
          RT0750,
          RT1500,
          RT3000,
          RT7500
        };
        enum MEAS_MODE {
          NORMAL_BIAS,
          POSITIVE_BIAS,
          NEGATIVE_BIAS
        };
        enum FS_MAG {
          FS088G,
          FS130G,
          FS190G,
          FS250G,
          FS400G,
          FS470G,
          FS560G,
          FS810G
        };
        enum OP_MODE {
          CONT_MEAS,
          SINGLE_MEAS,
          IDLE_MEAS
        };
        enum REG {
          CONFIG_A_REG,
          CONFIG_B_REG,
          MODE_REG,
          DATA_X_MSB_REG,
          DATA_X_LSB_REG,
          DATA_Z_MSB_REG,
          DATA_Z_LSB_REG,
          DATA_Y_MSB_REG,
          DATA_Y_LSB_REG,
          STATUS_REG,
          ID_A_REG,
          ID_B_REG,
          ID_C_REG
        };
      
      public:
        utilities::consumer< packets::mag_data<TOPIC_MOCAP> > rx_mocap_data;
        utilities::consumer< packets::mag_data<TOPIC_SIM> > rx_sim_data;
        utilities::producer< packets::mag_data<TOPIC_DEVICE> > tx_device_data;
        
      public:
        utilities::state_machine< states::mag_state::state<> > m_module_sm;
        globals::health_sm<TOPIC_HMC5883L> m_health_sm;
        
      private:
        static const float sc_scales[8];
        const utilities::i2c_port& mc_port;
        const uint8 mc_i2c_address;
        packets::mag_data<TOPIC_DEVICE> m_device_data;
        packets::mag_data<TOPIC_SIM> m_sim_data;
        packets::mag_data<TOPIC_MOCAP> m_mocap_data;
        params::hmc5883l_config<TOPIC_ONBOARD> m_config_data;
        params::hmc5883l_calib<TOPIC_ONBOARD> m_calib_data;
        float m_inv_scale;
        uint32 m_i_sample;
        mDxD* mp_mag_samples;
        
      public:
        hmc5883l(const utilities::i2c_port& port, uint32 exec_count);
      private:
        void step(float ts);
        void term();
        void idle();
        void device(float ts);
        void mocap(float ts);
        void sim(float ts);
        void calibrate(float ts);
      private:
        void initialize_variables();
        void load_config_data();
        void set_op_mode(OP_MODE op_mode) const;
        void set_config(AVG_SAMP avg_samp, RATE rate, MEAS_MODE meas_mode) const;
        void set_full_scale(FS_MAG fs_mag);
        bool id_device() const;
        bool update(mv3x1& mag_data);
    };
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
