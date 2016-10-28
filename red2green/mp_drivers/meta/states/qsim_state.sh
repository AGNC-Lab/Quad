$TYPE = state
$NAME = qsim_state

$STATES = {
  idle
  reset
  running
  |term|
}

$TRANSITIONS = {
  self
  idle -> term,reset
  reset -> running
  running -> idle,reset
  term -> idle
}
