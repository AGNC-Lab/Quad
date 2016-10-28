$TYPE = state
$NAME = ctrl_auth_mode

$STATES = {
  gcs_ctrl
  api_ctrl
}

$TRANSITIONS = {
  self
  gcs_ctrl -> api_ctrl
  api_ctrl -> gcs_ctrl
}
