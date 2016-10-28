$TYPE = packet
$NAME = ahrs_data

$TOPICS = {
  ahrs
}

$ELEMENTS = {
  float ts_last # last timestamp [s]
  float dt # time step used during current step [s]
  float x[7x1] # EKF a-posteriori state estimate vector
  float P[7x7] # EKF state estimate error covariance matrix
  uint32 imu_samples_collected # number of IMU samples collected
  uint32 mag_samples_collected # number of magnetometer samples collected
  packets::imu_data init_imu_data # IMU packet for EKF initialization
  packets::mag_data init_mag_data # magnetometer packet for EKF initialization
}
