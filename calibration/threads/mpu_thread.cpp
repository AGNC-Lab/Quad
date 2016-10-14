
#include "mpu_thread.h"
#include "MPU6050/dmp.h"

//Helper functions to parse the config file
void split_(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
}


vector<string> split_(const string &s, char delim) {
    vector<string> elems;
    split_(s, delim, elems);
    return elems;
}

void AccCalibParam(Vec3 *AccCalib, double *radius) {

    string line;
    vector<string> line_vec;

    //Get parameters for attitude controller
    ifstream myfile ("/home/root/AccCalib.txt");
    if (myfile.is_open()) {
		while (getline (myfile ,line)) {
		    line_vec = split_(line, ' ');
		    if (line_vec[0] == "bias_x") {
	            AccCalib->v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "bias_y") {
				AccCalib->v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "bias_z") {
				AccCalib->v[2] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "Radius") {
	            *radius = atof(line_vec[2].c_str());
		    }
		}
		myfile.close();
    }
    else {
		printf("Unable to open file /home/root/AccCalib.txt \n"); 
    }
    printf("Bias_x %f\nBias_y %f\nBias_z %f\nRadius %f\n",AccCalib->v[0],AccCalib->v[1],AccCalib->v[2],*radius);

	printf("Done loading accelerometer parameters\n");
}

void *IMU_Timer(void *threadID){

	printf("IMU_Timer has started!\n");
	int SamplingTime = 5;	//Sampling time in milliseconds

	//setup();

	while(1){
		WaitForEvent(e_Timeout,SamplingTime);


		//check if system should be terminated
		if(WaitForEvent(e_KeyESC,0) == 0){
			break;
		}

		SetEvent(e_IMU_trigger);

	}
	
	printf("IMU_Timer stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}


void *IMU_Task(void *threadID){
	printf("IMU_Task has started!\n");

	Vec3 IMU_localData_RPY;
	Vec4 IMU_localData_Quat;
	Vec4 IMU_localData_QuatNoYaw;
	Vec4 IMU_Quat_Yaw; //Quaternion with only yaw
	Vec4 IMU_Quat_Conversion; //When sitting on the ground, measured attitude indicates 180Deg Roll (need to unroll)
	IMU_Quat_Conversion.v[0] = cos(PI/2);
	IMU_Quat_Conversion.v[1] = sin(PI/2);
	IMU_Quat_Conversion.v[2] = 0;
	IMU_Quat_Conversion.v[3] = 0;


	int calibrate = 0;
	int cal_amount = 100;
	float Vel_Cal_X = 0;
	float Vel_Cal_Y = 0;
	float Vel_Cal_Z = 0;

	setup();

	while (calibrate < cal_amount) {
	    getDMP();
	    Vel_Cal_X +=  (float)gx/131;
	    Vel_Cal_Y +=  (float)gy/131;
	    Vel_Cal_Z +=  (float)gz/131;
	    calibrate++;
	}
	Vel_Cal_X /= cal_amount;
	Vel_Cal_Z /= cal_amount;
	Vel_Cal_Y /= cal_amount;

	Vec3 AccCalib;
	AccCalib.v[0] = 0; AccCalib.v[1] = 0; AccCalib.v[2] = 0;
	double radius = 8192;
	AccCalibParam(&AccCalib, &radius);


	while(1){
		
		WaitForEvent(e_IMU_trigger,500);

		if(WaitForEvent(e_KeyESC,0) == 0){
			break;
		}
		
		getDMP();
		IMU_localData_Quat.v[0] = q.w;
		IMU_localData_Quat.v[1] = q.y;
		IMU_localData_Quat.v[2] = q.x;
		IMU_localData_Quat.v[3] = -q.z; //Negative seemed necessary

		
		IMU_localData_Quat = QuaternionProduct(IMU_Quat_Conversion, IMU_localData_Quat); //Unroll vehicle		
		IMU_localData_RPY = Quat2RPY(IMU_localData_Quat);

		//Take off yaw from quaternion
		IMU_Quat_Yaw.v[0] = cos(IMU_localData_RPY.v[2]/2);
		IMU_Quat_Yaw.v[1] = 0;
		IMU_Quat_Yaw.v[2] = 0;
		IMU_Quat_Yaw.v[3] = -sin(IMU_localData_RPY.v[2]/2);
		IMU_localData_QuatNoYaw = QuaternionProduct(IMU_Quat_Yaw, IMU_localData_Quat);
		
		pthread_mutex_lock(&IMU_Mutex);
		IMU_Data_RPY = IMU_localData_RPY;
		IMU_Data_Quat = IMU_localData_Quat;
		IMU_Data_QuatNoYaw = IMU_localData_QuatNoYaw;
		IMU_Data_Accel.v[0] = (double)aa.x;
		IMU_Data_Accel.v[1] = (double)aa.y;
		IMU_Data_Accel.v[2] = (double)aa.z;
		IMU_Data_AngVel.v[1] = ((double)gx/131 - Vel_Cal_X)*PI/180;
		IMU_Data_AngVel.v[0] = ((double)gy/131 - Vel_Cal_Y)*PI/180;
		IMU_Data_AngVel.v[2] = -((double)gz/131 - Vel_Cal_Z)*PI/180;
		pthread_mutex_unlock(&IMU_Mutex);
	}
	
	printf("IMU_Task stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}

Eigen::VectorXd SolveLeastSquares(Eigen::MatrixXd A, Eigen::VectorXd y) 
{
	return A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);
}

Eigen::VectorXd SolveLinearSystem(Eigen::MatrixXd A, Eigen::VectorXd y){
	return A.colPivHouseholderQr().solve(y);
}


void *Calib_Sensors(void *threadID){

	printf("Calib_Sensors has started!\n");
	int SamplingTime = 10;	//Sampling time in milliseconds
	int NData = 100;
	Vec3 localIMU_Data_Accel;
	Eigen::MatrixXd A(6*NData,4);
	Eigen::VectorXd y(6*NData);

	//setup();

	WaitForEvent(e_Timeout,1000);
	printf("\n\nPush Enter to start calibrating accelerometer or ESC to exit.\n");

	while(1){
		WaitForEvent(e_Timeout,100);

		if(WaitForEvent(e_KeyESC,0) == 0){
			break;
		}
		
		if(WaitForEvent(e_KeyEnter, 0) == 0)
	    {	
			//Start calibration procedure
			int k = 0;

			printf("Place the quadcopter facing up and press Enter.\n");
			WaitForEvent(e_KeyEnter, -1); //Wait until user presses enter
			for(int i = 0; i < NData; i++){
				pthread_mutex_lock(&IMU_Mutex);
					A(k, 0) = 1;
					A(k, 1) = 2 * IMU_Data_Accel.v[0];
					A(k, 2) = 2 * IMU_Data_Accel.v[1];
					A(k, 3) = 2 * IMU_Data_Accel.v[2];
					y(k) = pow(IMU_Data_Accel.v[0], 2) + pow(IMU_Data_Accel.v[1], 2) + pow(IMU_Data_Accel.v[2], 2);
				pthread_mutex_unlock(&IMU_Mutex);
				k++;
				printf("Progress: %d/%d \n",k,6*NData);
				WaitForEvent(e_Timeout,SamplingTime);
			}

			printf("Place the quadcopter upside down and press Enter.\n");
			WaitForEvent(e_KeyEnter, -1); //Wait until user presses enter
			for(int i = 0; i < NData; i++){
				pthread_mutex_lock(&IMU_Mutex);
					A(k, 0) = 1;
					A(k, 1) = 2 * IMU_Data_Accel.v[0];
					A(k, 2) = 2 * IMU_Data_Accel.v[1];
					A(k, 3) = 2 * IMU_Data_Accel.v[2];
					y(k) = pow(IMU_Data_Accel.v[0], 2) + pow(IMU_Data_Accel.v[1], 2) + pow(IMU_Data_Accel.v[2], 2);
				pthread_mutex_unlock(&IMU_Mutex);
				k++;
				printf("Progress: %d/%d \n",k,6*NData);
				WaitForEvent(e_Timeout,SamplingTime);
			}

			printf("Roll the quadcopter 90 degrees and press Enter.\n");
			WaitForEvent(e_KeyEnter, -1); //Wait until user presses enter
			for(int i = 0; i < NData; i++){
				pthread_mutex_lock(&IMU_Mutex);
					A(k, 0) = 1;
					A(k, 1) = 2 * IMU_Data_Accel.v[0];
					A(k, 2) = 2 * IMU_Data_Accel.v[1];
					A(k, 3) = 2 * IMU_Data_Accel.v[2];
					y(k) = pow(IMU_Data_Accel.v[0], 2) + pow(IMU_Data_Accel.v[1], 2) + pow(IMU_Data_Accel.v[2], 2);
				pthread_mutex_unlock(&IMU_Mutex);
				k++;
				printf("Progress: %d/%d \n",k,6*NData);
				WaitForEvent(e_Timeout,SamplingTime);
			}

			printf("Roll the quadcopter -90 degrees and press Enter.\n");
			WaitForEvent(e_KeyEnter, -1); //Wait until user presses enter
			for(int i = 0; i < NData; i++){
				pthread_mutex_lock(&IMU_Mutex);
					A(k, 0) = 1;
					A(k, 1) = 2 * IMU_Data_Accel.v[0];
					A(k, 2) = 2 * IMU_Data_Accel.v[1];
					A(k, 3) = 2 * IMU_Data_Accel.v[2];
					y(k) = pow(IMU_Data_Accel.v[0], 2) + pow(IMU_Data_Accel.v[1], 2) + pow(IMU_Data_Accel.v[2], 2);
				pthread_mutex_unlock(&IMU_Mutex);
				k++;
				printf("Progress: %d/%d \n",k,6*NData);
				WaitForEvent(e_Timeout,SamplingTime);
			}

			printf("Pitch the quadcopter 90 degrees and press Enter.\n");
			WaitForEvent(e_KeyEnter, -1); //Wait until user presses enter
			for(int i = 0; i < NData; i++){
				pthread_mutex_lock(&IMU_Mutex);
					A(k, 0) = 1;
					A(k, 1) = 2 * IMU_Data_Accel.v[0];
					A(k, 2) = 2 * IMU_Data_Accel.v[1];
					A(k, 3) = 2 * IMU_Data_Accel.v[2];
					y(k) = pow(IMU_Data_Accel.v[0], 2) + pow(IMU_Data_Accel.v[1], 2) + pow(IMU_Data_Accel.v[2], 2);
				pthread_mutex_unlock(&IMU_Mutex);
				k++;
				printf("Progress: %d/%d \n",k,6*NData);
				WaitForEvent(e_Timeout,SamplingTime);
			}

			printf("Pitch the quadcopter -90 degrees and press Enter.\n");
			WaitForEvent(e_KeyEnter, -1); //Wait until user presses enter
			for(int i = 0; i < NData; i++){
				pthread_mutex_lock(&IMU_Mutex);
					A(k, 0) = 1;
					A(k, 1) = 2 * IMU_Data_Accel.v[0];
					A(k, 2) = 2 * IMU_Data_Accel.v[1];
					A(k, 3) = 2 * IMU_Data_Accel.v[2];
					y(k) = pow(IMU_Data_Accel.v[0], 2) + pow(IMU_Data_Accel.v[1], 2) + pow(IMU_Data_Accel.v[2], 2);
				pthread_mutex_unlock(&IMU_Mutex);
				k++;
				printf("Progress: %d/%d \n",k,6*NData);
				WaitForEvent(e_Timeout,SamplingTime);
			}

			Eigen::VectorXd X = SolveLeastSquares(A, y);
			Eigen::VectorXd bias(3);
			bias << X(1), X(2), X(3);
			double radius = X(0) + bias.squaredNorm();

			std::ofstream ofs ("AccCalib.txt", std::ofstream::out);

			ofs << "Accelerometer bias: \n";
			ofs << "bias_x = " << bias(0) << std::endl;
			ofs << "bias_y = " << bias(1) << std::endl;
			ofs << "bias_z = " << bias(2) << std::endl;
			ofs << "Accelerometer Radius: \n";
			ofs << "Radius = " <<  sqrt(radius) << std::endl << std::endl;

			ofs.close();

			
			printf("Calibration Procedure Finished!\n");



			SetEvent(e_KeyESC); //Terminate program
		}

	}
	
	printf("Calib_Sensors stopping...\n");
	threadCount -= 1;
	pthread_exit(NULL);
}