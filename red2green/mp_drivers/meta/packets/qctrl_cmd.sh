$TYPE = packet
$NAME = qctrl_cmd

$TOPICS = {
  gcs
  api
}

$ELEMENTS = {
  states::ctrl_auth_mode auth_src # control authority source, either GCS or API
  states::qctrl_mode mode # control mode
  float ch[10x1] # control channels:
                # mode = STANDBY_MODE
                # mode = MOTOR_TEST_MODE
                #   ch(0) - limited pulse-width of motor 1 [usec]
                #   ch(1) - limited pulse-width of motor 2 [usec]
                #   ch(2) - limited pulse-width of motor 3 [usec]
                #   ch(3) - limited pulse-width of motor 4 [usec]
                #   ch(4) - N/A
                #   ch(5) - N/A
                #   ch(6) - N/A
                #   ch(7) - N/A
                #   ch(8) - N/A
                #   ch(9) - N/A
                # mode = MOTOR_CTRL_MODE
                #   ch(0) - pulse-width of motor 1 [usec]
                #   ch(1) - pulse-width of motor 2 [usec]
                #   ch(2) - pulse-width of motor 3 [usec]
                #   ch(3) - pulse-width of motor 4 [usec]
                #   ch(4) - N/A
                #   ch(5) - N/A
                #   ch(6) - N/A
                #   ch(7) - N/A
                #   ch(8) - N/A
                #   ch(9) - N/A
                # mode = INERTIAL_CTRL_MODE_1
                #   ch(0) - x-body-rate [rad/s]
                #   ch(1) - y-body-rate [rad/s]
                #   ch(2) - z-body-rate [rad/s]
                #   ch(3) - collective pulse-width-rate [usec/s]
                #   ch(4) - N/A
                #   ch(5) - N/A
                #   ch(6) - N/A
                #   ch(7) - N/A
                #   ch(8) - N/A
                #   ch(9) - N/A
                # mode = INERTIAL_CTRL_MODE_2
                #   ch(0) - yaw-rate reference [rad/s]                
                #   ch(1) - pitch angle reference [rad]
                #   ch(2) - roll angle reference [rad]
                #   ch(3) - collective pulse-width-rate [usec/s]
                #   ch(4) - N/A
                #   ch(5) - N/A
                #   ch(6) - N/A
                #   ch(7) - N/A
                #   ch(8) - N/A
                #   ch(9) - N/A
                # mode = INERTIAL_CTRL_MODE_3
                #   ch(0) - yaw-rate reference [rad/s]
                #   ch(1) - pitch angle reference [rad]
                #   ch(2) - roll angle reference [rad]
                #   ch(3) - z-specific-thrust reference [m/s^2]
                #   ch(4) - N/A
                #   ch(5) - N/A
                #   ch(6) - N/A
                #   ch(7) - N/A
                #   ch(8) - N/A
                #   ch(9) - N/A
                # mode = TILT_COMP_MODE
                #   ch(0) - yaw-rate reference [rad/s]
                #   ch(1) - pitch angle reference [rad]
                #   ch(2) - roll angle reference [rad]
                #   ch(3) - tilt compensated down-acceleration reference [m/s^2]
                #   ch(4) - N/A
                #   ch(5) - N/A
                #   ch(6) - N/A
                #   ch(7) - N/A
                #   ch(8) - N/A
                #   ch(9) - N/A
                # mode = ALT_HOLD_MODE
                #   ch(0) - yaw-rate reference [rad/s]
                #   ch(1) - pitch angle reference [rad]
                #   ch(2) - roll angle reference [rad]
                #   ch(3) - down-velocity reference [m/s]
                #   ch(4) - N/A
                #   ch(5) - N/A
                #   ch(6) - N/A
                #   ch(7) - N/A
                #   ch(8) - N/A
                #   ch(9) - N/A
                # mode = VEL_CTRL_MODE
                #   ch(0) - yaw-rate reference [rad/s]
                #   ch(1) - forward-velocity reference [m/s]
                #   ch(2) - right-velocity reference [m/s]
                #   ch(3) - down-velocity reference [m/s]
                #   ch(4) - N/A
                #   ch(5) - N/A
                #   ch(6) - N/A
                #   ch(7) - N/A
                #   ch(8) - N/A
                #   ch(9) - N/A
                # mode = PVA_CTRL_MODE
                #   ch(0) - yaw-rate reference [rad/s]
                #   ch(1) - north-position reference [m]
                #   ch(2) - east-position reference [m]
                #   ch(3) - down-position reference [m]
                #   ch(4) - north-velocity reference [m/s]
                #   ch(5) - east-velocity reference [m/s]
                #   ch(6) - down-velocity reference [m/s]
                #   ch(7) - north-acceleration reference [m/s^2]
                #   ch(8) - east-acceleration reference [m/s^2]
                #   ch(9) - down-acceleration reference [m/s^2]
}
