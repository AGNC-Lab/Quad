$TYPE = params
$NAME = ahrs_params

$TOPICS = {
  onboard
  offboard
}

$PRIMARY_ELEMENTS = {
  float std_P_qq_0 # initial EKF quaternion state error standard deviation
  float std_P_bb_0_dps # initial EKF gyro bias state error standard deviation [deg/s]
  float std_gyro_dps # gyroscope measurement standard deviation [deg/s]
  float std_b_gyro_dps2 # gyroscope bias process noise standard deviation [deg/s^2]
  float std_pseudo_q_norm # quaternion norm pseudo-measurement standard deviation
  float std_grav_gs # gravity correction measurement standard deviation [g's]
  float std_hdg_deg # heading measurement standard deviation [deg]
  
  uint32 n_imu_samples # Number of IMU samples required for initialization.
  uint32 n_mag_samples # Number of magnetometer samples required for initialization.
  
  float tau # gyro bias time correlation factor [s]
  float grav_update_threshold_gs # gravity update threshold [g's]
  float max_hdg_innov_deg # maximum allowed heading innovation [deg]
  float mag_dec_deg # local magnetic declination angle [deg]
}

$SECONDARY_ELEMENTS = {
  var_P_qq_0 <- square <- std_P_qq_0 # initial EKF quaternion state error variance
  var_P_bb_0 <- square <- deg2rad <- std_P_bb_0_dps # init EKF gyro bias state err var [rad^2/s^2]
  var_gyro <- square <- deg2rad <- std_gyro_dps # gyroscope measurement variance [rad^2/s^2]
  var_b_gyro <- square <- deg2rad <- std_b_gyro_dps2 # gyro bias process noise variance [rad^2/s^4]
  var_pseudo_q_norm <- square <- std_pseudo_q_norm # quaternion norm pseudo-measurement variance
  var_grav_gs <- square <- std_grav_gs # gravity correction measurement variance [g's^2]
  var_hdg <- square <- deg2rad <- std_hdg_deg # heading measurement variance [rad^2]
  
  tau_inv <- inv <- tau # inverse of gyro bias time correlation factor [1/s]
  max_hdg_innov <- deg2rad <- max_hdg_innov_deg # maximum allowed heading innovation [rad]
  mag_dec <- deg2rad <- mag_dec_deg # local magnetic declination angle [rad]
}
