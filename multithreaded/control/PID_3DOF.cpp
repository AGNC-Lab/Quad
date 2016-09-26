

#include "PID_3DOF.h"

//Sets initial errors to zero
void initializePID(PID_3DOF* PID){
	Vec3 zeros;
	zeros.v[0] = 0; 
	zeros.v[1] = 0; 
	zeros.v[2] = 0;

	PID->e_prop = zeros;
	PID->e_deriv = zeros;
	PID->e_integ = zeros;
}

//Sets integral error to zero
void resetIntegralErrorPID(PID_3DOF* PID){
	PID->e_integ.v[0] = 0;
	PID->e_integ.v[1] = 0;
	PID->e_integ.v[2] = 0;
}

//Update Kp, Ki and Kd in the PID
void updateControlParamPID(PID_3DOF* PID, Vec3 K_p, Vec3 K_i, Vec3 K_d, Vec3 maxInteg){
	PID->K_p = K_p;
	PID->K_d = K_d;
	PID->K_i = K_i;
	PID->maxInteg = maxInteg;
}

//Update all errors
void updateErrorPID(PID_3DOF* PID, Vec3 feedForward, Vec3 e_prop, Vec3 e_deriv, float dt){

	PID->feedForward = feedForward;
	PID->e_prop = e_prop;
	PID->e_deriv = e_deriv;
	PID->e_integ = Add3x1Vec(PID->e_integ, ScaleVec3(e_prop, dt)); //e_integ = e_integ + e_prop*dt

	//Saturate integral error
	for (int i = 0; i < 3; i++){
		if (PID->e_integ.v[i] > PID->maxInteg.v[i]){
			PID->e_integ.v[i] = PID->maxInteg.v[i];
		}
		else if (PID->e_integ.v[i] < -PID->maxInteg.v[i]){
			PID->e_integ.v[i] = -PID->maxInteg.v[i];
		}
	}
}

//Calculate output of PID
Vec3 outputPID(PID_3DOF PID){
	Vec3 PID_out;
	for (int i = 0; i < 3; i++)
	{
		PID_out.v[i] =  PID.feedForward.v[i] + 
						PID.e_prop.v[i] * PID.K_p.v[i] + 
						PID.e_deriv.v[i] * PID.K_d.v[i] + 
						PID.e_integ.v[i] * PID.K_i.v[i];
	}

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
	Vec3 KP_RPY, KD_RPY, KI_RPY, maxInteg_RPY;
	Vec3 KP_w, KD_w, KI_w, maxInteg_w;

    string line;
    vector<string> line_vec;
    ifstream myfile ("/home/root/configAtt.txt");
    if (myfile.is_open()) {
		while (getline (myfile ,line)) {
		    line_vec = split(line, ' ');
		    if (line_vec[0] == "KP_R") {
	            KP_RPY.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_P") {
				KP_RPY.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_Y") {
				KP_RPY.v[2] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_R") {
	            	KD_RPY.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_P") {
				KD_RPY.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_Y") {
				KD_RPY.v[2] = atof(line_vec[2].c_str());
		    }
		    if (line_vec[0] == "KI_R") {
	            KI_RPY.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_P") {
				KI_RPY.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_Y") {
				KI_RPY.v[2] = atof(line_vec[2].c_str());
		    }
		    if (line_vec[0] == "maxInteg_R") {
	            maxInteg_RPY.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_P") {
				maxInteg_RPY.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_Y") {
				maxInteg_RPY.v[2] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_wx") {
				KP_w.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_wy") {
				KP_w.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KP_wz") {
		        KP_w.v[2]  = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_wx") {
				KD_w.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_wy") {
				KD_w.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KD_wz") {
		        KD_w.v[2]   = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_wx") {
				KI_w.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_wy") {
				KI_w.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "KI_wz") {
		        KI_w.v[2]  = atof(line_vec[2].c_str());
		    }
		    if (line_vec[0] == "maxInteg_wx") {
	            maxInteg_w.v[0] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_wy") {
				maxInteg_w.v[1] = atof(line_vec[2].c_str());
		    }
		    else if (line_vec[0] == "maxInteg_wz") {
				maxInteg_w.v[2] = atof(line_vec[2].c_str());
		    }
		}
		myfile.close();
		updateControlParamPID(PID_att, KP_RPY, KI_RPY, KD_RPY, maxInteg_RPY);
		updateControlParamPID(PID_angVel, KP_w, KI_w, KD_w, maxInteg_w);
    }
    
    else {
		printf("Unable to open file"); 
    }

	printf("Done updating control parameters\n");
}