////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/mp_ap_board.h                                                           //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FSW_HARDWARE_MP_AP_BOARD_H
#define FSW_HARDWARE_MP_AP_BOARD_H

#include "fsw/globals/object.h"
#include "fsw/hardware/hmc5883l.h"
#include "fsw/hardware/globals.h"
#include "fsw/hardware/mpu6050.h"
#include "fsw/hardware/overo.h"
#include "fsw/hardware/pca9685.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace fsw {
  namespace hardware {
    class mp_ap_board : public globals::object {
      public:
        enum IMU_MUX_STATE {
          SELECT_I2C,
          SELECT_SPI
        };
        enum MAG_SWITCH_STATE {
          SELECT_ONBOARD_MAG,
          SELECT_OFFBOARD_MAG
        };
        enum ALT_SWITCH_STATE {
          SELECT_ONBOARD_ALT,
          SELECT_OFFBOARD_ALT
        };
        
      private:
        const overo& mc_overo;
        params::mp_ap_board_config<TOPIC_ONBOARD> m_config_data;
        mpu6050 m_imu;
        hmc5883l m_mag;
        pca9685 m_pwm;
        
      public:
        mp_ap_board(const overo& device,
                    uint32 imu_exec_count,
                    uint32 mag_exec_count,
                    uint32 pwm_exec_count);
        mpu6050& imu();
        hmc5883l& mag();
        pca9685& pwm();
        void set_pwm_mux_to_ap() const;
      private:
        void load_config_data();
        void mpu6050_multiplexer(IMU_MUX_STATE state) const;
        void hmc5883l_switch(MAG_SWITCH_STATE state) const;
        void mpl3115a2_switch(ALT_SWITCH_STATE state) const;
    };
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
