$TYPE = state
$NAME = qctrl_mode

$STATES = {
  standby_mode
  motor_test_mode
  motor_ctrl_mode
  inertial_ctrl_mode_1
  inertial_ctrl_mode_2
  inertial_ctrl_mode_3
  tilt_comp_mode
  alt_hold_mode
  vel_ctrl_mode
  pva_ctrl_mode
}

$TRANSITIONS = {
  self
  standby_mode -> all
  motor_test_mode -> standby_mode
  motor_ctrl_mode -> all \ motor_test_mode
  inertial_ctrl_mode_1 -> all \ motor_test_mode
  inertial_ctrl_mode_2 -> all \ motor_test_mode
  inertial_ctrl_mode_3 -> all \ motor_test_mode
  tilt_comp_mode -> all \ motor_test_mode
  alt_hold_mode -> all \ motor_test_mode
  vel_ctrl_mode -> all \ motor_test_mode
  pva_ctrl_mode -> all \ motor_test_mode
}
