$TYPE = state
$NAME = health_state

$TOPICS = {
  hmc5883l
  mpu6050
}

$STATES = {
  green
  yellow
  red
}

$TRANSITIONS = {
  self
  green -> red
  yellow -> red
  red -> yellow
}
