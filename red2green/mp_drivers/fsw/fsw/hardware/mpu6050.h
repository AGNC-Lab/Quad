////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/mpu6050.h                                                               //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FSW_HARDWARE_MPU6050_H
#define FSW_HARDWARE_MPU6050_H

#include "fsw/globals/module.h"
#include "fsw/globals/health_sm.h"
#include "fsw/hardware/globals.h"
#include "packets/all.h"
#include "params/all.h"
#include "states/all.h"
#include "utilities/consumer.h"
#include "utilities/producer.h"
#include "utilities/i2c_port.h"
#include "utilities/state_machine.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace fsw {
  namespace hardware {
    class mpu6050 : public globals::module {
      public:
        enum ADDRESS_BIT {
          DEFAULT_ADDRESS=0b0000000,
          A0=0b0000001
        };
        enum DLPF_CFG {
          DLPF0,
          DLPF1,
          DLPF2,
          DLPF3,
          DLPF4,
          DLPF5,
          DLPF6
        };
        enum ACCL_FS {
          FS_2G,
          FS_4G,
          FS_8G,
          FS_16G
        };
        enum GYRO_FS {
          FS_250DPS,
          FS_500DPS,
          FS_1000DPS,
          FS_2000DPS
        };
        enum REG {
          WHO_AM_I_REG=0x75,
          PWR_MGMT_1_REG=0x6B,
          SMPRT_DIV_REG=0x19,
          CONFIG_REG=0x1A,
          GYRO_CONFIG_REG,
          ACCL_CONFIG_REG,
          ACCL_XOUT_H_REG=0x3B,
          ACCL_XOUT_L_REG,
          ACCL_YOUT_H_REG,
          ACCL_YOUT_L_REG,
          ACCL_ZOUT_H_REG,
          ACCL_ZOUT_L_REG,
          TEMP_OUT_H_REG,
          TEMP_OUT_L_REG,
          GYRO_XOUT_H_REG,
          GYRO_XOUT_L_REG,
          GYRO_YOUT_H_REG,
          GYRO_YOUT_L_REG,
          GYRO_ZOUT_H_REG,
          GYRO_ZOUT_L_REG
        };
        
      public:
        utilities::consumer< packets::imu_data<TOPIC_SIM> > rx_sim_data;
        utilities::producer< packets::imu_data<TOPIC_DEVICE> > tx_device_data;
        
      public:
        utilities::state_machine< states::imu_state::state<> > m_module_sm;
        globals::health_sm<TOPIC_MPU6050> m_health_sm;
        
      private:
        static const float sc_accl_scales[4];
        static const float sc_gyro_scales[4];
        const utilities::i2c_port& mc_port;
        const uint8 mc_i2c_base_address;
        const uint8 mc_i2c_address;
        const float mc_temp_coeff0;
        const float mc_temp_coeff1;
        packets::imu_data<TOPIC_DEVICE> m_device_data;
        packets::imu_data<TOPIC_SIM> m_sim_data;
        params::mpu6050_config<TOPIC_ONBOARD> m_config_data;
        params::mpu6050_calib<TOPIC_ONBOARD> m_calib_data;
        float m_inv_accl_scale;
        float m_inv_gyro_scale;
        uint32 m_i_sample;
        v3x1 m_accl_bias_sum;
        v3x1 m_gyro_bias_sum;
        
      public:
        mpu6050(const utilities::i2c_port& port, ADDRESS_BIT address_bit, uint32 exec_count);
      private:
        void step(float ts);
        void term();
        void idle();
        void device(float ts);
        void sim(float ts);
        void calibrate(float ts);
      private:
        void initialize_variables();
        void load_config_data();
        void set_clock_source() const;
        void set_dlpf_config(DLPF_CFG dlpf_cfg) const;
        void set_sample_rate_div(uint8 smprt_div) const;
        void set_accl_full_scale(ACCL_FS accl_fs);
        void set_gyro_full_scale(GYRO_FS gyro_fs);
        bool id_device() const;
        bool update_accl(mv3x1& accl_data);
        bool update_gyro(mv3x1& gyro_data);
        bool update_temp(float& temp_data);
        bool update(mv3x1& accl_data, mv3x1& gyro_data);
        bool update(mv3x1& accl_data, mv3x1& gyro_data, float& temp_data);
    };
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
