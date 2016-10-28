$TYPE = params
$NAME = pid_gains

$PRIMARY_ELEMENTS = {
  float k_p # proportional gain
  float k_i # integral gain
  float k_d # derivative gain
  float int_min # lower bound on integral
  float int_max # upper bound on integral
}