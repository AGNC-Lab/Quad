

//Functions

//Attitude error: e_r = 0.5*invSkew(Rref'*Rbw - Rbw'*Rref)
Vec3 AttitudeErrorVector(Mat3x3 Rbw, Mat3x3 Rdes);

//Attitude control inputs: u_att = -Kr*e_r - Kw*e_w
Vec3 AttitudeControlInputs(Mat3x3 Kr, Mat3x3 Kw, Vec3 e_r, Vec3 e_w);


/*Return inputs to quadrotor for attitude control
q[4];		//Attitude quaternion
Rdes;		//Desired rotation matrix
w_bw[3];	//Angular velocity of the system
wDes[3];	//Desired angular velocity
u1;			//Thrust 
This algorithm was extracted from Mellinger, 2011: Minimum Snap Trajectory Generation and Control for Quadrotors*/
Vec4 attitudeControl(Vec4 q, Mat3x3 Rdes, Vec3 w_bw, Vec3 wDes, float u1);

/* Convert inputs u = [Thrust, torque_roll, torque_pitch, torque_yaw] into pwm values (T coordinates)
T coordinates assumes: u = [KT    KT    KT   KT   ] [pwm1^2]
                            0     -KT*L 0    KT*L ] [pwm2^2]
                            -KT*L 0     KT*L 0    ] [pwm3^2]
                            KM    -KM   KM   -KM  ] [pwm4^2]    
KT = thrust coefficient for the propellers
KM = moment coefficient for the propellers */
Vec4 u2pwmTshape(Vec4 u);

/* Convert inputs u = [Thrust, torque_roll, torque_pitch, torque_yaw] into pwm values (X coordinates)
X coordinates assumes: u =[1    0      0       0 ] [KT    KT    KT   KT   ] [pwm1^2]
						  [0    cos(o) -sin(o) 0 ] [0     -KT*L 0    KT*L ] [pwm2^2]
						  [0    sin(o) cos(o)  0 ] [-KT*L 0     KT*L 0    ] [pwm3^2]
						  [0    0      0       1 ] [KM    -KM   KM   -KM  ] [pwm4^2]    
KT = thrust coefficient for the propellers
KM = moment coefficient for the propellers */
Vec4 u2pwmXshape(Vec4 u);
