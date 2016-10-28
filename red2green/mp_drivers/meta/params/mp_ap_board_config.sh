$TYPE = params
$NAME = mp_ap_board_config

$TOPICS = {
  onboard
  offboard
}

$PRIMARY_ELEMENTS = {
  uint8 imu_mux_state
  uint8 mag_switch_state
  uint8 alt_switch_state
}
