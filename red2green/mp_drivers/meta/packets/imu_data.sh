$TYPE = packet
$NAME = imu_data

$TOPICS = {
  device
  sim
  ahrs
}

$ELEMENTS = {
  float accl[3x1] # acceleration [m/s^2]
  float gyro[3x1] # angular velocity [rad/s]
}
