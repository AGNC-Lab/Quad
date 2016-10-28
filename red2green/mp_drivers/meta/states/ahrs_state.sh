$TYPE = state
$NAME = ahrs_state

$STATES = {
  idle
  reset
  running
  |term|
}

$TRANSITIONS = {
  self
  idle -> term,reset,running
  reset -> running
  running -> idle,reset
  term -> idle
}
