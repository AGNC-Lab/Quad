////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/quat.h                                                                      //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_QUAT_H
#define UTILITIES_QUAT_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/dcm.h"
#include "utilities/globals.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  template <class T> class dcm;

  template <class T> class quat {
    friend class dcm<T>;

    private:
      typedef Eigen::Matrix<T,3,1> v3x1T;
      typedef Eigen::Matrix<T,4,1> v4x1T;
      typedef Eigen::Matrix<T,4,3> m4x3T;

    private:
      T m_q[4];

    public:
      quat() {
        m_q[0] = 1.0;
        m_q[1] = 0.0;
        m_q[2] = 0.0;
        m_q[3] = 0.0;
      }
      quat(const quat<T>& q) {
        m_q[0] = q.m_q[0];
        m_q[1] = q.m_q[1];
        m_q[2] = q.m_q[2];
        m_q[3] = q.m_q[3];
      }
      quat(const dcm<T>& C) {
        operator()(C);
      }
      quat(T s, const v3x1T& v) {
        m_q[0] = s;
        m_q[1] = v(0);
        m_q[2] = v(1);
        m_q[3] = v(2);
      }
      quat(T q0, T q1, T q2, T q3) {
        m_q[0] = q0;
        m_q[1] = q1;
        m_q[2] = q2;
        m_q[3] = q3;
      }
      quat(T roll, T pitch, T yaw) {
        operator()(roll,pitch,yaw);
      }
      quat(const v3x1T& euler_axis) {
        operator()(euler_axis);
      }

      const T& w() const {
        return m_q[0];
      }
      const T& x() const {
        return m_q[1];
      }
      const T& y() const {
        return m_q[2];
      }
      const T& z() const {
        return m_q[3];
      }
      T& w() {
        return m_q[0];
      }
      T& x() {
        return m_q[1];
      }
      T& y() {
        return m_q[2];
      }
      T& z() {
        return m_q[3];
      }

      T scalar() const {
        return m_q[0];
      }
      v3x1T vector() const {
        return (v3x1T() << m_q[1],m_q[2],m_q[3]).finished();
      }
      v4x1T raw() const {
        return (v4x1T() << m_q[0],m_q[1],m_q[2],m_q[3]).finished();
      }
      quat<T> conjugate() const {
        // This function returns the conjugate of the calling quaternion.
        return quat<T>(m_q[0],-m_q[1],-m_q[2],-m_q[3]);
      }
      void rectify() {
        // This function forces the scalar component of the quaternion to be positive.
        T sign_val = static_cast<T>(m_q[0] >= 0.0)-static_cast<T>(m_q[0] < 0.0);
        m_q[0] *= sign_val;
        m_q[1] *= sign_val;
        m_q[2] *= sign_val;
        m_q[3] *= sign_val;
      }
      void normalize() {
        // This function forces the norm of m_q to equal unity.
        T mag = sqrt(m_q[0]*m_q[0]+m_q[1]*m_q[1]+m_q[2]*m_q[2]+m_q[3]*m_q[3]);
        T mag_inv = 1.0/mag;
        m_q[0] *= mag_inv;
        m_q[1] *= mag_inv;
        m_q[2] *= mag_inv;
        m_q[3] *= mag_inv;
      }
      m4x3T E() const {
        // This function returns the rate matrix of the quaternion, such that dqdt = E(q_A2B)*w, 
        // where w is a 3x1 vector of body-rates of the B frame relative to the A frame, resolved 
        // in B frame coordinates, and dqdt is the time derivative of q_A2B.
        T q0 = 0.5*m_q[0];
        T q1 = 0.5*m_q[1];
        T q2 = 0.5*m_q[2];
        T q3 = 0.5*m_q[3];

        return (m4x3T() << -q1,-q2,-q3,
                            q0,-q3, q2,
                            q3, q0,-q1,
                           -q2, q1, q0).finished();
      }

      T roll() const {
        // This function should be used only if the quaternion calling this function describes the 
        // attitude of a frame (B) relative to a local navigation frame (N). The output of this 
        // function is in radians.

        // C_n2b_23 = sin(phi)*cos(theta).
        T C_n2b_23 = 2.0*(m_q[2]*m_q[3]+m_q[0]*m_q[1]);

        // C_n2b_33 = cos(phi)*cos(theta).
        T C_n2b_33 = 1.0-2.0*(m_q[1]*m_q[1]+m_q[2]*m_q[2]);

        // Compute roll: C_n2b_23/C_n2b_33 = tan(phi).
        return atan2(C_n2b_23,C_n2b_33);
      }
      T pitch() const {
        // This function should be used only if the quaternion calling this function describes the 
        // attitude of a frame (B) relative to a local navigation frame (N). The output of this 
        // function is in radians.

        // C_n2b_13 = -sin(theta).
        T C_n2b_13 = 2.0*(m_q[1]*m_q[3]-m_q[0]*m_q[2]);

        // Bound C_n2b_13 to the interval [-1,1].
        T C_n2b_13_bounded = bound(static_cast<T>(-1.0),C_n2b_13,static_cast<T>(1.0));

        // Compute pitch: -C_n2b_13 = sin(theta).
        return asin(-C_n2b_13_bounded);
      }
      T yaw() const {
        // This function should be used only if the quaternion calling this function describes the 
        // attitude of a frame (B) relative to a local navigation frame (N). The output of this 
        // function is in radians.

        // C_n2b_11 = cos(theta)*cos(psi).
        T C_n2b_11 = 1.0-2.0*(m_q[2]*m_q[2]+m_q[3]*m_q[3]);

        // C_n2b_12 = cos(theta)*sin(psi).
        T C_n2b_12 = 2.0*(m_q[1]*m_q[2]+m_q[0]*m_q[3]);

        // Compute yaw: C_n2b_12/C_n2b_11 = tan(psi).
        return atan2(C_n2b_12,C_n2b_11);
      }
      T roll_deg() const {
        return roll()*constants::rad2deg;
      }
      T pitch_deg() const {
        return pitch()*constants::rad2deg;
      }
      T yaw_deg() const {
        return yaw()*constants::rad2deg;
      }

      quat<T>& operator()(const dcm<T>& C) {
        T tr0 = 1.0+C.m_mat[0]+C.m_mat[4]+C.m_mat[8];
        T tr1 = 1.0+C.m_mat[0]-C.m_mat[4]-C.m_mat[8];
        T tr2 = 1.0-C.m_mat[0]+C.m_mat[4]-C.m_mat[8];
        T tr3 = 1.0-C.m_mat[0]-C.m_mat[4]+C.m_mat[8];

        if ((tr0 >= tr1)&&(tr0 >= tr2)&&(tr0 >= tr3)) {
          T S = 2.0*sqrt(tr0);
          T S_inv = 1.0/S;

          m_q[0] = 0.25*S;
          m_q[1] = (C.m_mat[5]-C.m_mat[7])*S_inv;
          m_q[2] = (C.m_mat[6]-C.m_mat[2])*S_inv;
          m_q[3] = (C.m_mat[1]-C.m_mat[3])*S_inv;
        }
        else if ((tr1 >= tr0)&&(tr1 >= tr2)&&(tr1 >= tr3)) {
          T S = 2.0*sqrt(tr1);
          T S_inv = 1.0/S;

          m_q[0] = (C.m_mat[5]-C.m_mat[7])*S_inv;
          m_q[1] = 0.25*S;
          m_q[2] = (C.m_mat[1]+C.m_mat[3])*S_inv;
          m_q[3] = (C.m_mat[2]+C.m_mat[6])*S_inv;
        }
        else if ((tr2 >= tr0)&&(tr2 >= tr1)&&(tr2 >= tr3)) {
          T S = 2.0*sqrt(tr2);
          T S_inv = 1.0/S;

          m_q[0] = (C.m_mat[6]-C.m_mat[2])*S_inv;
          m_q[1] = (C.m_mat[1]+C.m_mat[3])*S_inv;
          m_q[2] = 0.25*S;
          m_q[3] = (C.m_mat[5]+C.m_mat[7])*S_inv;
        }
        else {
          T S = 2.0*sqrt(tr3);
          T S_inv = 1.0/S;

          m_q[0] = (C.m_mat[1]-C.m_mat[3])*S_inv;
          m_q[1] = (C.m_mat[2]+C.m_mat[6])*S_inv;
          m_q[2] = (C.m_mat[5]+C.m_mat[7])*S_inv;
          m_q[3] = 0.25*S;
        }

        return *this;
      }
      quat<T>& operator()(T roll, T pitch, T yaw) {
        operator()(dcm<T>(roll,pitch,yaw));
        return *this;
      }
      quat<T>& operator()(const v3x1T& euler_axis) {
        // This function accepts an Euler axis of non-unity magnitude. The magnitude of the 
        // Euler axis is used to extract the scalar component of the quaternion. The normalized 
        // Euler axis is used to create the vector part of the quaternion. If the Euler axis 
        // is has a magnitude of zero, then assume m_q = [1 0 0 0]^T.

        // Compute Euler axis norm.
        T e_norm = euler_axis.norm();

        // Compute scalar component of quaternion.
        m_q[0] = cos(0.5*e_norm);

        // Check that Euler axis had non-zero magnitude.
        if (e_norm > 0.0) {
          // Compute vector component of quaternion.
          T coeff = sin(0.5*e_norm)/e_norm;
          m_q[1] = coeff*euler_axis(0);
          m_q[2] = coeff*euler_axis(1);
          m_q[3] = coeff*euler_axis(2);
        }
        else {
          // If Euler axis had magnitude of zero, set vector part of quaternion to zero.
          m_q[1] = 0.0;
          m_q[2] = 0.0;
          m_q[3] = 0.0;
        }

        return *this;
      }
      quat<T>& operator=(const quat<T>& q) {
        m_q[0] = q.m_q[0];
        m_q[1] = q.m_q[1];
        m_q[2] = q.m_q[2];
        m_q[3] = q.m_q[3];
        return *this;
      }
      quat<T> operator*(const quat<T>& q) const {
        // This function performs quaternion multiplication between the calling 
        // quaternion and q, and returns a new quaternion.
        return quat(m_q[0]*q.m_q[0]-m_q[1]*q.m_q[1]-m_q[2]*q.m_q[2]-m_q[3]*q.m_q[3],
                    m_q[1]*q.m_q[0]+m_q[0]*q.m_q[1]+m_q[3]*q.m_q[2]-m_q[2]*q.m_q[3],
                    m_q[2]*q.m_q[0]-m_q[3]*q.m_q[1]+m_q[0]*q.m_q[2]+m_q[1]*q.m_q[3],
                    m_q[3]*q.m_q[0]+m_q[2]*q.m_q[1]-m_q[1]*q.m_q[2]+m_q[0]*q.m_q[3]);
      }

      friend std::ostream& operator<<(std::ostream& stream, const quat<T>& q) {
        stream << std::fixed
               << std::showpos
               << std::setprecision(4)
               << std::setw(6) << q.m_q[0] << ","
               << std::setw(6) << q.m_q[1] << ","
               << std::setw(6) << q.m_q[2] << ","
               << std::setw(6) << q.m_q[3];
        return stream;
      }
  };

//  typedef quat<float> quat;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
