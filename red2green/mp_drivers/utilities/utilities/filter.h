////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/filter.h                                                                    //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_FILTER_H
#define UTILITIES_FILTER_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  template <uint32 N_NUM, uint32 N_DEN> class filter {
    public:
      typedef Eigen::Array<float,1,N_NUM> v_num;
      typedef Eigen::Array<float,1,N_DEN> v_den;
      typedef Eigen::Array<float,1,N_NUM-1> v_num_1;
      typedef Eigen::Array<float,1,N_DEN-1> v_den_1;

    private:
      v_num m_num;
      v_den m_den;
      v_num m_x;
      v_den m_y;

    public:
      filter() {
        m_num = v_num::Zero();
        m_den = v_den::Zero();
        m_x = v_num::Zero();
        m_y = v_den::Zero();
      }
      void reset() {
        m_x = v_num::Zero();
        m_y = v_den::Zero();
      }
      float step(float x_k) {
        m_x(0) = x_k;
        float x_sum = (m_num*m_x).sum();
        float y_sum = ((m_den.template segment<N_DEN-1>(1))*
                         (m_y.template segment<N_DEN-1>(1))).sum();
        m_y(0) = (x_sum-y_sum)/m_den(0);

        v_num_1 x_temp = m_x.template segment<N_NUM-1>(0);
        v_den_1 y_temp = m_y.template segment<N_DEN-1>(0);

        m_x.template segment<N_NUM-1>(1) = x_temp;
        m_y.template segment<N_DEN-1>(1) = y_temp;

        return m_y(0);
      }
      float& num(uint32 i) {
        return m_num(i);
      }
      float& den(uint32 i) {
        return m_den(i);
      }
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
