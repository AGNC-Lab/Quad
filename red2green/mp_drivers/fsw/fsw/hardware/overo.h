////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/overo.h                                                                 //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FSW_HARDWARE_OVERO_H
#define FSW_HARDWARE_OVERO_H

#include "fsw/globals/object.h"
#include "fsw/hardware/globals.h"
#include "utilities/consumer.h"
#include "utilities/producer.h"
#include "utilities/i2c_port.h"
#include "utilities/mutex.h"
#include "utilities/thread.h"
#include "utilities/uart_port.h"
#include "params/all.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace fsw {
  namespace hardware {
    class overo : public globals::object {
      public:
        enum GPIO_CH {
          GPIO_10,
          GPIO_64,
          GPIO_65,
          GPIO_114,
          GPIO_144,
          GPIO_145,
          GPIO_146,
          GPIO_147,
          GPIO_168,
          GPIO_186
        };
        enum GPIO_IRQ_CH {
          GPIO_IRQ_10,
          GPIO_IRQ_64,
          GPIO_IRQ_65,
          GPIO_IRQ_114,
          GPIO_IRQ_146,
          GPIO_IRQ_147,
          GPIO_IRQ_186
        };
        enum GPIO_DIRECTION {
          IN,
          OUT
        };
        enum GPIO_TRIGGER {
          RISING,
          FALLING,
          BOTH
        };
        enum GPIO_VALUE {
          LOW,
          HIGH
        };
        
      private:
        class pw_monitor : public utilities::thread {
          public:
            utilities::producer<float> tx_pw_usec;
            
          private:
            GPIO_IRQ_CH m_irq_ch;
            uint32 m_timeout_msec;
            float m_pw_filt_factor;
            
          public:
            pw_monitor(GPIO_IRQ_CH irq_ch, uint32 timeout_msec, float pw_filt_factor);
            ~pw_monitor();
            void execute();
        };
        
      private:
        static const char* sc_ch[];
        static const char* sc_irq_ch[];
        static const char* sc_dir[];
        static const char* sc_trig[];
        static const char* sc_val[];
        static pw_monitor* s_pw_monitor_pointers[];
        static uint32 s_pw_monitor_counter[];
        static uint32 s_instantiations;
        static utilities::mutex s_mutex;
        
      public:
        static void gpio_config_for_pw_monitoring(GPIO_IRQ_CH irq_ch);
        static void gpio_set_direction(GPIO_CH ch, GPIO_DIRECTION dir);
        static void gpio_set_trigger(GPIO_CH ch, GPIO_TRIGGER trig);
        static void gpio_set_value(GPIO_CH ch, GPIO_VALUE val);
        static bool gpio_get_value(GPIO_CH ch);
        static utilities::producer<float>& launch_pw_monitor(GPIO_IRQ_CH irq_ch);
        static utilities::producer<float>& terminate_pw_monitor(GPIO_IRQ_CH irq_ch);
        
      private:
        const std::string mc_config_filename;
        params::overo_config<TOPIC_ONBOARD> m_config_data;
        utilities::i2c_port m_i2c_port;
        utilities::uart_port m_uart_port;
        uint32 m_timeout_msec;
        float m_pw_filt_factor;
        
      public:
        overo();
        ~overo();
        const utilities::i2c_port& i2c_port() const;
        const utilities::uart_port& uart_port() const;
    };
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
