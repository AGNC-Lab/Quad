$TYPE = state
$NAME = qctrl_state

$STATES = {
  idle
  running
  |term|
}

$TRANSITIONS = {
  self
  idle -> term,running
  running -> idle
  term -> idle
}
