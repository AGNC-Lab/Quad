$TYPE = params
$NAME = pca9685_config

$TOPICS = {
  onboard
  offboard
}

$PRIMARY_ELEMENTS = {
  uint32 n_calib_samples
  float calibration_pulse_width_usec
  float calibration_update_rate
  float update_rate
  float min_pw_usec[6x1]
  float max_pw_usec[6x1]
  float channel_offset[6x1]
}
