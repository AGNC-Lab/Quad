$TYPE = params
$NAME = mpu6050_config

$TOPICS = {
  onboard
  offboard
}

$PRIMARY_ELEMENTS = {
  uint8 dlpf_cfg
  uint8 sample_rate_div
  uint8 accl_fs
  uint8 gyro_fs
  uint32 n_calib_samples
  float C_dev2body[3x3]
}
