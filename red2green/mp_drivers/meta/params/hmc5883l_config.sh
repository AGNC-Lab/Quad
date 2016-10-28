$TYPE = params
$NAME = hmc5883l_config

$TOPICS = {
  onboard
  offboard
}

$PRIMARY_ELEMENTS = {
  uint8 op_mode
  uint8 avg_samp
  uint8 rate
  uint8 meas_mode
  uint8 fs_mag
  uint32 n_calib_samples
  float C_dev2body[3x3]
}
