$TYPE = state
$NAME = mag_state

$STATES = {
  idle
  device
  mocap
  sim
  calibrate
  |term|
}

$TRANSITIONS = {
  self
  idle -> term,device,mocap,sim
  device -> idle,mocap,calibrate
  mocap -> idle,device
  sim -> idle
  calibrate -> device
  term -> idle
}
