////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE:   utilities/dcm.h                                                                       //
// AUTHORS: Miki Szmuk                                                                            //
// PI:      Behcet Acikmese                                                                       //
// LAB:     Autonomous Controls Lab (ACL)                                                         //
// LICENSE: Copyright 2016, All Rights Reserved                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_DCM_H
#define UTILITIES_DCM_H

////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utilities/globals.h"
#include "utilities/quat.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace utilities {
  template <class T> class quat;

  template <class T> class dcm {
    friend class quat<T>;

    private:
      typedef Eigen::Matrix<T,3,1> v3x1T;
      typedef Eigen::Matrix<T,3,3> m3x3T;

    private:
      T m_mat[9];

    public:
      static dcm<T> R1(T angle) {
        dcm<T> C;
        T s = sin(angle);
        T c = cos(angle);

        C.m_mat[0] = 1.0;
        C.m_mat[1] = 0.0;
        C.m_mat[2] = 0.0;
        C.m_mat[3] = 0.0;
        C.m_mat[4] = c;
        C.m_mat[5] = s;
        C.m_mat[6] = 0.0;
        C.m_mat[7] = -s;
        C.m_mat[8] = c;

        return C;
      }
      static dcm<T> R2(T angle) {
        dcm<T> C;
        T s = sin(angle);
        T c = cos(angle);

        C.m_mat[0] = c;
        C.m_mat[1] = 0.0;
        C.m_mat[2] = -s;
        C.m_mat[3] = 0.0;
        C.m_mat[4] = 1.0;
        C.m_mat[5] = 0.0;
        C.m_mat[6] = s;
        C.m_mat[7] = 0.0;
        C.m_mat[8] = c;

        return C;
      }
      static dcm<T> R3(T angle) {
        dcm<T> C;
        T s = sin(angle);
        T c = cos(angle);

        C.m_mat[0] = c;
        C.m_mat[1] = s;
        C.m_mat[2] = 0.0;
        C.m_mat[3] = -s;
        C.m_mat[4] = c;
        C.m_mat[5] = 0.0;
        C.m_mat[6] = 0.0;
        C.m_mat[7] = 0.0;
        C.m_mat[8] = 1.0;

        return C;
      }

      dcm() {
        m_mat[0] = 1.0;
        m_mat[1] = 0.0;
        m_mat[2] = 0.0;
        m_mat[3] = 0.0;
        m_mat[4] = 1.0;
        m_mat[5] = 0.0;
        m_mat[6] = 0.0;
        m_mat[7] = 0.0;
        m_mat[8] = 1.0;
      }
      dcm(const dcm<T>& C) {
        m_mat [0] = C.m_mat[0];
        m_mat [1] = C.m_mat[1];
        m_mat [2] = C.m_mat[2];
        m_mat [3] = C.m_mat[3];
        m_mat [4] = C.m_mat[4];
        m_mat [5] = C.m_mat[5];
        m_mat [6] = C.m_mat[6];
        m_mat [7] = C.m_mat[7];
        m_mat [8] = C.m_mat[8];
      }
      dcm(const quat<T>& q) {
        operator()(q);
      }
      dcm(T s, const v3x1T& v) {
        operator()(quat<T>(s,v));
      }
      dcm(T q0, T q1, T q2, T q3) {
        operator()(quat<T>(q0,q1,q2,q3));
      }
      dcm(T roll, T pitch, T yaw) {
        operator()(roll,pitch,yaw);
      }
      dcm(const v3x1T& euler_axis) {
        operator()(euler_axis);
      }

      const T& xx() const {
        return m_mat[0];
      }
      const T& xy() const {
        return m_mat[1];
      }
      const T& xz() const {
        return m_mat[2];
      }
      const T& yx() const {
        return m_mat[3];
      }
      const T& yy() const {
        return m_mat[4];
      }
      const T& yz() const {
        return m_mat[5];
      }
      const T& zx() const {
        return m_mat[6];
      }
      const T& zy() const {
        return m_mat[7];
      }
      const T& zz() const {
        return m_mat[8];
      }
      T& xx() {
        return m_mat[0];
      }
      T& xy() {
        return m_mat[1];
      }
      T& xz() {
        return m_mat[2];
      }
      T& yx() {
        return m_mat[3];
      }
      T& yy() {
        return m_mat[4];
      }
      T& yz() {
        return m_mat[5];
      }
      T& zx() {
        return m_mat[6];
      }
      T& zy() {
        return m_mat[7];
      }
      T& zz() {
        return m_mat[8];
      }

      T trace() const {
        return m_mat[0]+m_mat[4]+m_mat[8];
      }
      m3x3T raw() const {
        return (m3x3T() << m_mat[0],m_mat[1],m_mat[2],
                           m_mat[3],m_mat[4],m_mat[5],
                           m_mat[6],m_mat[7],m_mat[8]);
      }
      dcm<T> transpose() const {
        dcm<T> C_T;

        C_T.m_mat[0] = m_mat[0];
        C_T.m_mat[1] = m_mat[3];
        C_T.m_mat[2] = m_mat[6];
        C_T.m_mat[3] = m_mat[1];
        C_T.m_mat[4] = m_mat[4];
        C_T.m_mat[5] = m_mat[7];
        C_T.m_mat[6] = m_mat[2];
        C_T.m_mat[7] = m_mat[5];
        C_T.m_mat[8] = m_mat[8];

        return *this;
      }

      T roll() const {
        // This function should be used only if the quaternion calling this function describes the 
        // attitude of a frame (B) relative to a local navigation frame (N).

        // Compute roll: C_n2b_23/C_n2b_33 = tan(phi).
        return atan2(m_mat[5],m_mat[8]);
      };
      T pitch() const {
        // This function should be used only if the quaternion calling this function describes the 
        // attitude of a frame (B) relative to a local navigation frame (N).

        // Bound C_n2b_13 to the interval [-1,1].
        T C_n2b_13_bounded = bound(static_cast<T>(-1.0),m_mat[2],static_cast<T>(1.0));

        // Compute pitch: -C_n2b_13 = sin(theta).
        return asin(-C_n2b_13_bounded);
      }
      T yaw() const {
        // This function should be used only if the quaternion calling this function describes the 
        // attitude of a frame (B) relative to a local navigation frame (N).

        // Compute yaw: C_n2b_12/C_n2b_11 = tan(psi).
        return atan2(m_mat[1],m_mat[0]);
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

      dcm<T>& operator()(const quat<T>& q) {
        // Compute DCM entries from quaternion q. This function assumes norm(q) = 1.
        m_mat[0] = 1.0-2.0*(q.m_q[2]*q.m_q[2]+q.m_q[3]*q.m_q[3]);
        m_mat[1] = 2.0*(q.m_q[1]*q.m_q[2]+q.m_q[0]*q.m_q[3]);
        m_mat[2] = 2.0*(q.m_q[1]*q.m_q[3]-q.m_q[0]*q.m_q[2]);
        m_mat[3] = 2.0*(q.m_q[1]*q.m_q[2]-q.m_q[0]*q.m_q[3]);
        m_mat[4] = 1.0-2.0*(q.m_q[1]*q.m_q[1]+q.m_q[3]*q.m_q[3]);
        m_mat[5] = 2.0*(q.m_q[2]*q.m_q[3]+q.m_q[0]*q.m_q[1]);
        m_mat[6] = 2.0*(q.m_q[1]*q.m_q[3]+q.m_q[0]*q.m_q[2]);
        m_mat[7] = 2.0*(q.m_q[2]*q.m_q[3]-q.m_q[0]*q.m_q[1]);
        m_mat[8] = 1.0-2.0*(q.m_q[1]*q.m_q[1]+q.m_q[2]*q.m_q[2]);
        return *this;
      }
      dcm<T>& operator()(T roll, T pitch, T yaw) {
        // Pre-compute sin and cos of roll, pitch, and yaw.
        T sin_phi = sin(roll);
        T cos_phi = cos(roll);
        T sin_theta = sin(pitch);
        T cos_theta = cos(pitch);
        T sin_psi = sin(yaw);
        T cos_psi = cos(yaw);

        // Compute DCM entries. These computations assume that the order of rotations is yaw (3), 
        // pitch (2), and roll (1). The matrix that is computed gives the transformation from the 
        // local navigation frame to the body frame.
        m_mat[0] = cos_theta*cos_psi;
        m_mat[1] = cos_theta*sin_psi;
        m_mat[2] = -sin_theta;
        m_mat[3] = sin_phi*sin_theta*cos_psi-cos_phi*sin_psi;
        m_mat[4] = sin_phi*sin_theta*sin_psi+cos_phi*cos_psi;
        m_mat[5] = sin_phi*cos_theta;
        m_mat[6] = cos_phi*sin_theta*cos_psi+sin_phi*sin_psi;
        m_mat[7] = cos_phi*sin_theta*sin_psi-sin_phi*cos_psi;
        m_mat[8] = cos_phi*cos_theta;

        return *this;
      }
      dcm<T>& operator()(const v3x1T& euler_axis) {
        operator()(quat<T>(euler_axis));
        return *this;
      }
      dcm<T>& operator=(const dcm& C) {
        m_mat[0] = C.m_mat[0];
        m_mat[1] = C.m_mat[1];
        m_mat[2] = C.m_mat[2];
        m_mat[3] = C.m_mat[3];
        m_mat[4] = C.m_mat[4];
        m_mat[5] = C.m_mat[5];
        m_mat[6] = C.m_mat[6];
        m_mat[7] = C.m_mat[7];
        m_mat[8] = C.m_mat[8];
        return *this;
      }
      dcm<T> operator*(const dcm& C) const {
        dcm res;

        res.m_mat[0] = m_mat[0]*C.m_mat[0]+m_mat[1]*C.m_mat[3]+m_mat[2]*C.m_mat[6];
        res.m_mat[1] = m_mat[0]*C.m_mat[1]+m_mat[1]*C.m_mat[4]+m_mat[2]*C.m_mat[7];
        res.m_mat[2] = m_mat[0]*C.m_mat[2]+m_mat[1]*C.m_mat[5]+m_mat[2]*C.m_mat[8];
        res.m_mat[3] = m_mat[3]*C.m_mat[0]+m_mat[4]*C.m_mat[3]+m_mat[5]*C.m_mat[6];
        res.m_mat[4] = m_mat[3]*C.m_mat[1]+m_mat[4]*C.m_mat[4]+m_mat[5]*C.m_mat[7];
        res.m_mat[5] = m_mat[3]*C.m_mat[2]+m_mat[4]*C.m_mat[5]+m_mat[5]*C.m_mat[8];
        res.m_mat[6] = m_mat[6]*C.m_mat[0]+m_mat[7]*C.m_mat[3]+m_mat[8]*C.m_mat[6];
        res.m_mat[7] = m_mat[6]*C.m_mat[1]+m_mat[7]*C.m_mat[4]+m_mat[8]*C.m_mat[7];
        res.m_mat[8] = m_mat[6]*C.m_mat[2]+m_mat[7]*C.m_mat[5]+m_mat[8]*C.m_mat[8];

        return res;
      }
      v3x1T operator*(const v3x1T& v) const {
        v3x1T res;

        res(0) = m_mat[0]*v(0)
                +m_mat[1]*v(1)
                +m_mat[2]*v(2);
        res(1) = m_mat[3]*v(0)
                +m_mat[4]*v(1)
                +m_mat[5]*v(2);
        res(2) = m_mat[6]*v(0)
                +m_mat[7]*v(1)
                +m_mat[8]*v(2);

        return res;
      }

      friend std::ostream& operator<<(std::ostream& stream, const dcm<T>& C) {
        stream << std::fixed
               << std::showpos
               << std::setprecision(4)
               << std::setw(6) << C.m_mat[0] << ","
               << std::setw(6) << C.m_mat[1] << ","
               << std::setw(6) << C.m_mat[2] << ","
               << std::setw(6) << C.m_mat[3] << ","
               << std::setw(6) << C.m_mat[4] << ","
               << std::setw(6) << C.m_mat[5] << ","
               << std::setw(6) << C.m_mat[6] << ","
               << std::setw(6) << C.m_mat[7] << ","
               << std::setw(6) << C.m_mat[8];
        return stream;
      }
  };

//  typedef dcm<float> dcm;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
