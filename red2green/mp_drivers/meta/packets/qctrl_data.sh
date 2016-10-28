$TYPE = packet
$NAME = qctrl_data

$TOPICS = {
  qctrl
}

$ELEMENTS = {
  float ts_last # last timestamp [s]
  float dt # time step used during current step [s]
  float pwm_coll # PWM collective value 
  float hdg_ref # heading reference value [rad]
  float pos_ref_ned[3x1] # position control reference vector in NED coordinates [m]
  float vel_ned[3x1] # numerically differentiated velocity vector in NED coordinates [m/s]
  packets::pid_data Ts_n_pid # specific-thrust north PID data packet
  packets::pid_data Ts_e_pid # specific-thrust east PID data packet
  packets::pid_data Ts_z_pid # specific-thrust z-axis PID data packet
  packets::pid_data omega_x_pid # omega_x PID data packet
  packets::pid_data omega_y_pid # omega_y PID data packet
  packets::pid_data omega_z_pid # omega_z PID data packet
}
