////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:    fsw/hardware/overo.cpp                                                               //
// DATE:     Tue Nov 24 03:26:36 2015                                                             //
// AUTHORS:  Michael (Miki) Szmuk, Tim Lowery                                                     //
// PI:       Dr. Behcet Acikmese                                                                  //
// LAB:      Autonomous GN&C Lab                                                                  //
// LICENSE:  Copyright 2015, Copyright 2016                                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <fcntl.h>
#include "fsw/hardware/overo.h"
#include <poll.h>
#include <sys/time.h>
using fsw::hardware::overo;
using std::ostringstream;
using std::string;
using utilities::i2c_port;
using utilities::mutex;
using utilities::producer;
using utilities::uart_port;

////////////////////////////////////////////////////////////////////////////////////////////////////

const char* overo::sc_ch[] = {
  "10",
  "64",
  "65",
  "114",
  "144",
  "145",
  "146",
  "147",
  "168",
  "186"
};
const char* overo::sc_irq_ch[] = {
  "10",
  "64",
  "65",
  "114",
  "146",
  "147",
  "186"
};
const char* overo::sc_dir[] = {
  "in",
  "out"
};
const char* overo::sc_trig[] = {
  "rising",
  "falling",
  "both"
};
const char* overo::sc_val[] = {
  "0",
  "1"
};
overo::pw_monitor* overo::s_pw_monitor_pointers[] = {
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL
};
uint32 overo::s_pw_monitor_counter[] = {
  0,
  0,
  0,
  0,
  0,
  0,
  0
};
uint32 overo::s_instantiations = 0;
mutex overo::s_mutex;

overo::pw_monitor::pw_monitor(GPIO_IRQ_CH irq_ch, uint32 timeout_msec, float pw_filt_factor) :
  thread("fsw::hardware::overo::pw_monitor(GPIO_IRQ_"+string(sc_irq_ch[irq_ch])+")",PR1),
  tx_pw_usec(name()+"::tx_pw_usec") {
  // Initialize local parameters.
  m_irq_ch = irq_ch;
  m_timeout_msec = timeout_msec;
  m_pw_filt_factor = pw_filt_factor;
}
overo::pw_monitor::~pw_monitor() {
  terminate();
}
void overo::pw_monitor::execute() {
  string filename = "/sys/class/gpio/gpio"+string(sc_irq_ch[m_irq_ch])+"/value";
  
  char read_buff[100]={0,};
  int fd = open(filename.c_str(),O_RDONLY);
  if (fd == -1) {
    utilities::error(name()+"::execute","failed to open file",errno);
  }
  
  struct pollfd pfd;
  pfd.fd = fd;
  pfd.events = POLLPRI|POLLERR;
  
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME,&ts);
  
  uint64 t_current_nsec = ts.tv_sec*1000000000+ts.tv_nsec;
  uint64 t_last_nsec = 0;
  uint64 pw_nsec = 0;
  float pw_filt_usec = 1500.0f;
  
  while(keep_running()) {
    lseek(fd,0,SEEK_SET);
    
    if (poll(&pfd,1,m_timeout_msec) == -1) {
      utilities::error(name()+"::execute","poll failed",errno);
    }
    else {
      if (read(fd,read_buff,1) == -1) {
        utilities::error(name()+"::execute","read failed",errno);
      }
      else {
        clock_gettime(CLOCK_REALTIME,&ts);
        t_last_nsec = t_current_nsec;
        t_current_nsec = ts.tv_sec*1000000000+ts.tv_nsec;
        pw_nsec = t_current_nsec-t_last_nsec;
        
        // Only register data when data is low and pulse with is less than 2500 us.
        if (pw_nsec < 2500000) {
          if (read_buff[0] == '0') {
            float pw_usec = 0.001f*((float)pw_nsec);
            pw_filt_usec = m_pw_filt_factor*pw_filt_usec+(1.0-m_pw_filt_factor)*pw_usec;
            
            // Push filtered pulse width data.
            tx_pw_usec.push(pw_filt_usec);
          }
        }
      }
    }
  }
  
  if (close(fd) == -1) {
    utilities::error(name()+"::execute","close failed",errno);
  }
}

void overo::gpio_config_for_pw_monitoring(GPIO_IRQ_CH irq_ch) {
  switch (irq_ch) {
    case GPIO_IRQ_10:
      gpio_set_direction(GPIO_10,IN);
      gpio_set_value(GPIO_10,LOW);
      gpio_set_trigger(GPIO_10,BOTH);
      break;
    case GPIO_IRQ_64:
      gpio_set_direction(GPIO_64,IN);
      gpio_set_value(GPIO_64,LOW);
      gpio_set_trigger(GPIO_64,BOTH);
      break;
    case GPIO_IRQ_65:
      gpio_set_value(GPIO_65,LOW);
      gpio_set_trigger(GPIO_65,BOTH);
      break;
    case GPIO_IRQ_114:
      gpio_set_value(GPIO_144,LOW);
      gpio_set_trigger(GPIO_114,BOTH);
    case GPIO_IRQ_146:
      gpio_set_direction(GPIO_146,IN);
      gpio_set_value(GPIO_146,LOW);
      gpio_set_trigger(GPIO_146,BOTH);
      break;
    case GPIO_IRQ_147:
      gpio_set_direction(GPIO_147,IN);
      gpio_set_value(GPIO_147,LOW);
      gpio_set_trigger(GPIO_147,BOTH);
      break;
    case GPIO_IRQ_186:
      gpio_set_direction(GPIO_186,IN);
      gpio_set_value(GPIO_186,LOW);
      gpio_set_trigger(GPIO_186,BOTH);
  }
}
void overo::gpio_set_direction(GPIO_CH ch, GPIO_DIRECTION dir) {
  if ((ch == GPIO_10)||(ch == GPIO_64)||(ch == GPIO_146)||(ch == GPIO_147)) {
    // Valid for 10,64,146,147,186
    string cmd = "echo "+
                 string(sc_dir[dir])+
                 " > /sys/class/gpio/gpio"+
                 string(sc_ch[ch])+
                 "/direction";
    system(cmd.c_str());
  }
  else {
    ostringstream ss;
    ss << "invalid channel input: " << (uint32)ch;
    utilities::warning("fsw::hardware::overo::gpio_set_direction",ss.str());
  }
}
void overo::gpio_set_trigger(GPIO_CH ch, GPIO_TRIGGER trig) {
  if ((ch == GPIO_10)||(ch == GPIO_64)||(ch == GPIO_65)||(ch == GPIO_114)||
      (ch == GPIO_146)||(ch == GPIO_147)||(ch == GPIO_186)) {
    // Valid for 10,64,65,114,146,147,186
    string cmd = "echo "+
                 string(sc_trig[trig])+
                 " > /sys/class/gpio/gpio"+
                 string(sc_ch[ch])+
                 "/edge";
    system(cmd.c_str());
  }
  else {
    ostringstream ss;
    ss << "invalid channel input: " << (uint32)ch;
    utilities::warning("fsw::hardware::overo::gpio_set_trigger",ss.str());
  }
}
void overo::gpio_set_value(GPIO_CH ch, GPIO_VALUE val) {
  string cmd = "echo "+
               string(sc_val[val])+
               " > /sys/class/gpio/gpio"+
               string(sc_ch[ch])+
               "/value";
  system(cmd.c_str());
}
bool overo::gpio_get_value(GPIO_CH ch) {
  // WIP
  return (ch == ch);/*DEBUG*/
}
producer<float>& overo::launch_pw_monitor(GPIO_IRQ_CH irq_ch) {
  s_mutex.lock();
    if (s_instantiations == 0) {
      utilities::warning("fsw::hardware::overo::allocate_pw_monitor",
        "cannot allocate pw_monitor without overo instantiation");
    }
    else {
      if (s_pw_monitor_counter[irq_ch]++ == 0) {
        gpio_config_for_pw_monitoring(irq_ch);
        s_pw_monitor_pointers[irq_ch]->launch();
      }
    }
  s_mutex.unlock();
  
  return s_pw_monitor_pointers[irq_ch]->tx_pw_usec;
}
producer<float>& overo::terminate_pw_monitor(GPIO_IRQ_CH irq_ch) {
  s_mutex.lock();
    if (s_instantiations == 0) {
      utilities::warning("fsw::hardware::overo::destroy_pw_monitor",
        "cannot destroy pw_monitor without overo instantiation");
    }
    else {
      if (--s_pw_monitor_counter[irq_ch] == 0) {
        s_pw_monitor_pointers[irq_ch]->terminate();
      }
    }
  s_mutex.unlock();
  
  return s_pw_monitor_pointers[irq_ch]->tx_pw_usec;
}

overo::overo() :
  object("hardware","overo"),
  mc_config_filename("../config/overo_config.txt"),
  m_i2c_port("/dev/i2c-3"),
  m_uart_port("/dev/ttyO0",uart_port::BR38400) {
  s_mutex.lock();
    if (s_instantiations++ == 0) {
      m_config_data.load(mc_config_filename);
      
      for (uint32 irq_ch=GPIO_IRQ_10; irq_ch != GPIO_IRQ_186; ++irq_ch) {
        s_pw_monitor_pointers[irq_ch] =
          new pw_monitor((GPIO_IRQ_CH)irq_ch,m_timeout_msec,m_pw_filt_factor);
      }
    }
    else {
      warning("overo","instantiated multiple hardware::overo");
    }
  s_mutex.unlock();
}
overo::~overo() {
  s_mutex.lock();
    if (--s_instantiations == 0) {
      for (uint32 irq_ch=GPIO_IRQ_10; irq_ch != GPIO_IRQ_186; ++irq_ch) {
        if (s_pw_monitor_counter[irq_ch] > 0) {
          warning("~overo","cannot delete pw_monitor that is in use");
        }
        else {
          delete s_pw_monitor_pointers[irq_ch];
          s_pw_monitor_pointers[irq_ch] = NULL;
        }
      }
    }
  s_mutex.unlock();
}
const i2c_port& overo::i2c_port() const {
  return m_i2c_port;
}
const uart_port& overo::uart_port() const {
  return m_uart_port;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
