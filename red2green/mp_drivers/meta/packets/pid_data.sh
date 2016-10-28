$TYPE = packet
$NAME = pid_data

$ELEMENTS = {
  float reset # reset coefficient (0.0 = reset, 1.0 = no reset)
  float u_p # proportional control effort
  float u_i # integral control effort
  float u_d # derivative control effort
  float u # total control effort
  float e # error used in time step
}
