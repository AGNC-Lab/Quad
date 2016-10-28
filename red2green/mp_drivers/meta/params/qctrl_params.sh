$TYPE = params
$NAME = qctrl_params

$TOPICS = {
  onboard
  offboard
}

$PRIMARY_ELEMENTS = {
  float cmd_timeout # length of time before controller transitions to standby 
                    # when no command is received [s]
  
  float zero_thrust_pwm # zero-thrust pulse-width [us]
  float max_test_delta_pw # maximum allowed delta pulse-width in MOTOR_TEST_MODE [0-1000] [us]
  float max_delta_pw # maximum allowed delta pulse-width in MOTOR_CTRL_MODE [0-1000] [us]
  float min_coll_pw # minimum allowed collective pulse-width contribution [us]
  float max_coll_pw # maximum allowed collective pulse-width contribution [us]
  float max_att_pw # maximum allowed attitude pulse-width contribution [us]
  
  float max_tilt_deg # maximum allowed tilt angle [deg]
  float max_load_factor # maximum allowed load factor
  float max_pwm_coll_rate # maximum allowed PWM collective rate [us/s]
  float max_hdg_rate_dps # maximum allowed heading rate [deg/s]
  float min_Ts_gs # minimum allowed reference specific thrust [g's]
  float max_Ts_gs # maximum allowed reference specific thrust [g's]
  float max_delta_Ts_gs # maximum allowed reference specific thrust control increment [g's]
  float max_omega_xy_dps # maximum allowed x- and y-axis body-rate [deg/s]
  float max_omega_z_dps # maximum allowed z-axis body-rate [deg/s]
  float att_p_gain # attitude control proportional gain
  
  float wn_ne # horizontal control natural frequency
  float zeta_ne # horizontal control damping ratio
  float wn_d # vertical control natural frequency
  float zeta_d # vertical control damping ratio
  
  float min_altitude # minimum altitude [m]
  float max_altitude # maximum altitude [m]
  float max_north_pos # maximum north position [m]
  float max_south_pos # maximum south position [m]
  float max_east_pos # maximum east position [m]
  float max_west_pos # maximum west position [m]
  float max_hvel # maximum horizontal velocity [m/s]
  float max_vvel # maximum vertical velocity [m/s]

  params::pid_gains Ts_ne_pid # specific thrust north-east PID gains
  params::pid_gains Ts_z_pid # specific thrust z-body PID gains
  params::pid_gains omega_xy_pid # xy body-rate PID gains
  params::pid_gains omega_z_pid # z body-rate PID gains
}

$SECONDARY_ELEMENTS = {
  max_tilt <- deg2rad <- max_tilt_deg # maximum allowed tilt angle [rad]
  max_hdg_rate <- deg2rad <- max_hdg_rate_dps # maximum allowed heading rate [rad/s]
  min_Ts <- g0 <- min_Ts_gs # minimum allowed reference specific thrust [m/s^2]
  max_Ts <- g0 <- max_Ts_gs # maximum allowed reference specific thrust [m/s^2]
  max_delta_Ts <- g0 <- max_delta_Ts_gs # max allowed ref specific thrust control increment [m/s^2]
  max_omega_xy <- deg2rad <- max_omega_xy_dps # maximum allowed x- and y-axis body-rate [rad/s]
  max_omega_z <- deg2rad <- max_omega_z_dps # maximum allowed z-axis body-rate [rad/s]
}
