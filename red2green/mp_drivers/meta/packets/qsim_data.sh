$TYPE = packet
$NAME = qsim_data

$TOPICS = {
  qsim
}

$ELEMENTS = {
  float ts_last # last timestamp [s]
  float dt # time step used during current step [s]
  double x[17x1] # simulation state vector
}
