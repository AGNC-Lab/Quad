$TYPE = params
$NAME = mpu6050_calib

$TOPICS = {
  onboard
  offboard
}

$PRIMARY_ELEMENTS = {
  float accl_bias[3x1]
  float gyro_bias[3x1]
}
