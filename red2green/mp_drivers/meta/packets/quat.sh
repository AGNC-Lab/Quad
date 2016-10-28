$TYPE = packet
$NAME = quat

$TOPICS = {
  ahrs
  qsim
}

$INCLUDES = {
  utilities/quat.h
}

$ELEMENTS = {
  utilities::quat<float> q # unit attitude quaternion
}

$INIT = {
  q = utilities::quat<float>();
}