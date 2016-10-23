

#include "PID_3DOF.h"

//Sets initial errors to zero
void initializePID(PID_3DOF* PID){
	Matrix<float, 3, 1> zeros = Matrix<float, 3, 1>::Zero(3, 1);

	PID->e_prop = zeros;
	PID->e_deriv = zeros;
	PID->e_integ = zeros;
}

//Sets integral error to zero
void resetIntegralErrorPID(PID_3DOF* PID){
	PID->e_integ = Matrix<float, 3, 1>::Zero(3, 1);
}

//Update Kp, Ki and Kd in the PID
void updateControlParamPID(PID_3DOF* PID, Matrix<float, 3, 1> K_p, Matrix<float, 3, 1> K_i, Matrix<float, 3, 1> K_d, Matrix<float, 3, 1> maxInteg){
	PID->K_p = K_p;
	PID->K_d = K_d;
	PID->K_i = K_i;
	PID->maxInteg = maxInteg;
}

//Update all errors
void updateErrorPID(PID_3DOF* PID, Matrix<float, 3, 1> feedForward, Matrix<float, 3, 1> e_prop, Matrix<float, 3, 1> e_deriv, float dt){

	PID->feedForward = feedForward;
	PID->e_prop = e_prop;
	PID->e_deriv = e_deriv;
	PID->e_integ = PID->e_integ+(e_prop*dt); //e_integ = e_integ + e_prop*dt

	//Saturate integral error
	for (int i = 0; i < 3; i++){
		if (PID->e_integ(i) > PID->maxInteg(i)){
			PID->e_integ(i) = PID->maxInteg(i);
		}
		else if (PID->e_integ(i) < -PID->maxInteg(i)){
			PID->e_integ(i) = -PID->maxInteg(i);
		}
	}
}

//Calculate output of PID
Matrix<float, 3, 1> outputPID(PID_3DOF PID){
	Matrix<float, 3, 1> PID_out;
	PID_out =  PID.feedForward + 
				PID.e_prop.cwiseProduct(PID.K_p) + 
				PID.e_deriv.cwiseProduct(PID.K_d) + 
				PID.e_integ.cwiseProduct(PID.K_i);		
	return PID_out;
}

//Helper functions to parse the config file
void split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
}


vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

void updatePar(PID_3DOF *PID_att, PID_3DOF *PID_angVel, PID_3DOF *PID_pos) {
	Matrix<float, 3, 1> KP_RPY, KD_RPY, KI_RPY, maxInteg_RPY;
	Matrix<float, 3, 1> KP_w, KD_w, KI_w, maxInteg_w;
	Matrix<float, 3, 1> KP_Pos, KD_Pos, KI_Pos, maxInteg_Pos;

    string line;
    vector<string> line_vec;

    //Get parameters for attitude controller
    ifstream myfile ("/home/root/configAtt.txt");
    if (myfile.is_open()) {
		while (getline (myfile ,line)) {
		    line_vec = split(line, ' ');
		    if (line_vec[0] == "KP_R") {
	            KP_RPY(0) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_P") {
				KP_RPY(1) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_Y") {
				KP_RPY(2) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_R") {
	            	KD_RPY(0) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_P") {
				KD_RPY(1) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_Y") {
				KD_RPY(2) = atof(line_vec[2].c_str());
		    }
		    if (line_vec[0] == "KI_R") {
	            KI_RPY(0) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_P") {
				KI_RPY(1) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_Y") {
				KI_RPY(2) = atof(line_vec[2].c_str());
		    }
		    if (line_vec[0] == "maxInteg_R") {
	            maxInteg_RPY(0) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_P") {
				maxInteg_RPY(1) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_Y") {
				maxInteg_RPY(2) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_wx") {
				KP_w(0) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_wy") {
				KP_w(1) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_wz") {
		        KP_w(2)  = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_wx") {
				KD_w(0) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_wy") {
				KD_w(1) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_wz") {
		        KD_w(2) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_wx") {
				KI_w(0) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_wy") {
				KI_w(1) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_wz") {
		        KI_w(2) = atof(line_vec[2].c_str());
		    }
		    if (line_vec[0] == "maxInteg_wx") {
	            maxInteg_w(0) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_wy") {
				maxInteg_w(1) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_wz") {
				maxInteg_w(2) = atof(line_vec[2].c_str());
		    }
		}
		myfile.close();
		updateControlParamPID(PID_att, KP_RPY, KI_RPY, KD_RPY, maxInteg_RPY);
		updateControlParamPID(PID_angVel, KP_w, KI_w, KD_w, maxInteg_w);
    }
    else {
		printf("Unable to open file /home/root/configAtt.txt \n"); 
    }

    //Get parameters for position controller
    ifstream myfile2 ("/home/root/configPos.txt");
    if (myfile2.is_open()) {
		while (getline (myfile2 ,line)) {
		    line_vec = split(line, ' ');
		    if (line_vec[0] == "KP_X") {
	            KP_Pos(0) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_Y") {
				KP_Pos(1) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_Z") {
				KP_Pos(2) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_X") {
	            KD_Pos(0) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_Y") {
				KD_Pos(1) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_Z") {
				KD_Pos(2) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_X") {
	            KI_Pos(0) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_Y") {
				KI_Pos(1) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_Z") {
				KI_Pos(2) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_X") {
	            maxInteg_Pos(0) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_Y") {
				maxInteg_Pos(1) = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_Z") {
				maxInteg_Pos(2) = atof(line_vec[2].c_str());
		    }
		}
		myfile2.close();
		updateControlParamPID(PID_pos, KP_Pos, KI_Pos, KD_Pos, maxInteg_Pos);
    }
    else {
		printf("Unable to open file /home/root/configPos.txt \n"); 
    }

    PrintVec3(PID_att->K_p, "PID_att Kp");
    PrintVec3(PID_att->K_i, "PID_att Ki");
    PrintVec3(PID_att->K_d, "PID_att Kd");
    PrintVec3(PID_angVel->K_p, "PID_w Kp");
    PrintVec3(PID_angVel->K_i, "PID_w Ki");
    PrintVec3(PID_angVel->K_d, "PID_w Kd");
    PrintVec3(PID_pos->K_p, "PID_pos Kp");
    PrintVec3(PID_pos->K_i, "PID_pos Ki");
    PrintVec3(PID_pos->K_d, "PID_pos Kd");

	printf("Done updating control parameters\n");
}