$TYPE = state
$NAME = imu_state

$STATES = {
  idle
  device
  sim
  calibrate
  |term|
}

$TRANSITIONS = {
  self
  idle -> term,device,sim
  device -> idle,calibrate
  sim -> idle
  calibrate -> device
  term -> idle
}
