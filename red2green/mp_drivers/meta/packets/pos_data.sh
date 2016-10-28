$TYPE = packet
$NAME = pos_data

$TOPICS = {
  mocap
  gps
  sim
}

$ELEMENTS = {
  float pos_ned[3x1] # position in local NED coordinate frame [m]
}
