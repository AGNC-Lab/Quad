$TYPE = packet
$NAME = pwm_cmd

$TOPICS = {
  device
  sim
}

$ELEMENTS = {
  float ch[6x1] # channel pulse widths [900,2100] [usec]
}
