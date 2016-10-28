$TYPE = params
$NAME = qsim_params

$TOPICS = {
  onboard
  offboard
}

$PRIMARY_ELEMENTS = {
  float r_i_ned[3x1] # Initial position in NED frame [m]
  float v_i_ned[3x1] # Initial velocity in NED frame [m/s]
  float roll_i_deg # Initial roll angle [deg]
  float pitch_i_deg # Initial pitch angle [deg]
  float yaw_i_deg # Initial yaw angle [deg]
  float omega_i_dps[3x1] # Initial body rate [deg/s]
  float m # vehicle mass [kg]
  float L # boom moment arm [m]
  float cT # propeller thrust coefficient
  float cM2cT # ratio of propeller moment to propeller thrust coefficient [m]
  float S # drag reference area [m^2]
  float cD_force # vehicle drag force coefficient
  float cD_moment # vehicle drag torque coefficient
  float rho # air density [kg/m^3]
  float tau_up # motor throttle-up time constant [s]
  float tau_down # motor throttle-down time constant [s]
  float J[3x3] # vehicle inertia tensor [kg*m^2]
}

$SECONDARY_ELEMENTS = {
  roll_i <- deg2rad <- roll_i_deg
  pitch_i <- deg2rad <- pitch_i_deg
  yaw_i <- deg2rad <- yaw_i_deg
  omega_i <- deg2rad <- omega_i_dps
  m_inv <- inv <- m
  cT2cM <- inv <- cM2cT
  tau_up_inv <- inv <- tau_up
  tau_down_inv <- inv <- tau_down
}
