#include <learning.h>


Learning::Learning() { }

Learning::~Learning() { }

#define CTRL_DEG2RAD  (M_PI/180.0)


bool Learning::configure(yarp::os::ResourceFinder &rf) {

	Network yarp;

	period = rf.check("period",Value(0.0001)).asDouble();
    entropy = rf.check("entropy",Value("off")).asString().c_str();
    type = rf.check("type",Value("batch")).asString().c_str();
    aw = rf.check("aw",Value("off")).asString().c_str();
    param = rf.check("param",Value("fixed")).asString().c_str();

	printf("Learning Parameters: [Type] %s [Entropy] %s [Anti-Windup] %s.\n\n", type.c_str(), entropy.c_str(), aw.c_str());
	printf("Estimation Parameters are : %s. \n\n", param.c_str());


	// open all ports
    bool ret = commandPort.open("/learning/rpc");
    ret &= commandSensorsPort.open("/learning/sensor/rpc");

    if(!ret) {
        yError()<<"Cannot open some of the ports";
        return false;
    }


    if(!attach(commandPort)) { // Connect to respond method
        yError()<<"Cannot attach to the commandPort";
        return false;
    }


    // Setting environment in order to get Jacobian
	
	arm = new iCub::iKin::iCubArm ("left");
	
	arm->setAllConstraints(false);   // we don't need to update the limits from the robot to compute the Jacobian
	arm->releaseLink(0);   // release torso pitch joint
	arm->releaseLink(1);   // release torso roll joint
	arm->releaseLink(2);   // release torso yaw joint


	finger = new iCub::iKin::iCubFinger ("left_index");

	Matrix H0(4,4);

    H0(0,0)=0.898138; H0(0,1)=0.439714;  H0(0,2)=0.0;      H0(0,3)=0.0022;
    H0(1,0)=-0.43804; H0(1,1)=0.89472;   H0(1,2)=0.087156; H0(1,3)=-0.021320433;
    H0(2,0)=0.038324; H0(2,1)=-0.078278; H0(2,2)=0.996195; H0(2,3)=0.005;
    H0(3,0)=0.0;      H0(3,1)=0.0;       H0(3,2)=0.0;      H0(3,3)=1.0;

    H0(2,0)=-H0(2,0);
    H0(2,1)=-H0(2,1);
    H0(1,2)=-H0(1,2);
    H0(2,3)=-H0(2,3);
    finger->setH0(H0);


    //Observation and dynamic model parameters
	if (type  == "batch"){

		H_k = zeros(7,7); //Rows depend on number of trials, columns on number of DoF
		err = zeros(7); //Depend on number of trials
		err_exp=zeros(7);
		Rt = 0.000025*eye(7,7); //Depend on number of trials
	}else if(type == "online" || type == "pbatch"){
		H_k = zeros(1,7); //Rows depend on number of trials, columns on number of DoF
		err = zeros(1); //Depend on number of trials
		err_exp = zeros(1);
		Rt = 0.000025*eye(1,1); //Depend on number of trials

	}

	Qt = 0.0015*eye(7,7); //Depend on number of DoF
	//Qt = 0.017*eye(7,7); //Depend on number of DoF
	Pd = 0.001*eye(7,7); //Used for AW (var_est -> Pd)
	
	//Rt = 0.000025*eye(7,7); //Depend on number of trials
	//Qt = 0.0009*eye(7,7); //Depend on number of DoF
	
	eye_7 = eye(7,7);

	//Estimation parameters
	u_est = zeros(7);
	//var_est = 0.03*eye(7,7);
	var_est = 0.03*eye(7,7);
	var_est_ant = var_est;

	

	u_bel = zeros(7);
	var_bel = zeros(7,7);
	
	bias = zeros(16);
	defineBias();


	n_vector.resize(3);
	aux = true;

    reading = 1; //number of readings between estimation spetps
    reading_t = 0; //total number of readings provided by controller
    current_reading = reading; //number of readings provided by controller
    startControlFlag = false;

    experiment=2;

    myfile = new std::ofstream();
	myfile->open ("u_est_1.txt");
	*myfile << u_est.toString().c_str();
	*myfile << "\n";

	if (param == "slow"){
		param_var = new std::ofstream();
		param_var->open ("param_var_1.txt");
		*param_var << bias.subVector(0,6).toString().c_str();
		*param_var << "\n";
	}
}

double Learning::getPeriod() {
    return period; // module periodicity (seconds)
}

bool Learning::updateModule() {

	Network yarp;

	if (startControlFlag == false){

		printf("No surface detect by sensors \n");
		Time::delay(1);
		return true;
	}


	if((type  == "batch" && reading % 8 == 0) || (type  == "online" && current_reading != reading) || (type  == "pbatch" && current_reading != reading)) { //Perform one filter iteration
		printf("Ready\n");


		// Prediction
		u_bel = u_est;
		var_bel = var_est + Qt;

		printf(" \n========================================\n");
		printf("var_bel is: \n(%s)\n", var_bel.toString().c_str());
		printf(" ========================================\n\n ");

		// Kalman Gain
		//Matrix pre_inv;
		//Matrix inv;
//
		//pre_inv = (H_k*var_bel*H_k.transposed() + Rt);

		//printf("pre_inv is: \n(%s)\n", pre_inv.toString().c_str());

		//inv = luinv(pre_inv);

		//printf("inv is: \n(%s)\n", inv.toString().c_str());

		Matrix K_gain = var_bel*H_k.transposed()*luinv((H_k*var_bel*H_k.transposed() + Rt));
		printf(" \n========================================\n");
		printf("K_gain is: \n(%s)\n", K_gain.toString().c_str());
		printf(" ========================================\n\n ");
		
		// Update
		var_est = (eye_7 - (K_gain*H_k)) * var_bel;
		if (type  == "batch"){
			u_est = u_bel + K_gain*(err_exp - err);	
			printf(" \n========================================\n");
			printf("u_est is: \n(%s)\n", u_est.toString().c_str());
			printf(" ========================================\n\n ");
			printf(" \n========================================\n");
			printf("var_est is: \n(%s)\n", var_est.toString().c_str());
			printf(" ========================================\n\n ");
			H_k = zeros(7,7); //Rows depend on number of trials, columns on number of DoF
			err = zeros(7); //Depend on number of trials

			reading = 1;
		}  
		else if (type  == "online"){

			if(entropy == "off"){

				u_est = u_bel + K_gain*(err_exp - err);	
				printf(" \n========================================\n");
				printf("u_est is: \n(%s)\n", u_est.toString().c_str());
				printf(" ========================================\n\n ");
				printf(" \n========================================\n");
				printf("var_est is: \n(%s)\n", var_est.toString().c_str());
				printf(" ========================================\n\n ");

				if (aw == "on"){
			    	Qt = (Pd*H_k.transposed()*H_k*Pd);
			    	Matrix Qt_det = luinv((H_k*Pd*H_k.transposed() + Rt));
			    	Qt = Qt*Qt_det(0,0);
					printf("[AW] Next Qt will be: \n(%s)\n\n", Qt.toString().c_str());
	
			    }

				H_k = zeros(1,7); //Rows depend on number of trials, columns on number of DoF
				err = zeros(1); //Depend on number of trials


		    }else{

				long double det1 = det(var_est);
				long double det2 = det(var_est_ant);

			    long double u = 0.5*log(det2/det1);

				//printf(" \n========================================\n");
				//printf("var_est_ant is: \n(%s)\n", var_est_ant.toString().c_str());
				//printf(" ========================================\n\n ");
	//
	//			//printf(" \n========================================\n");
	//			//printf("var_est is: \n(%s)\n", var_est.toString().c_str());
				//printf(" ========================================\n\n ");
		    

		    	printf("u is %.5Le\n", u);
		    	if(u >= -0.2){
		    		printf("Data is suficient to perform an estimation step!\n");		
		    	    u_est = u_bel + K_gain*(err_exp - err); 
		    	    var_est_ant = var_est;
	
			    	    printf(" \n========================================\n");
						printf("u_est is: \n(%s)\n", u_est.toString().c_str());
						printf(" ========================================\n\n ");
						printf(" \n========================================\n");
						printf("var_est is: \n(%s)\n", var_est.toString().c_str());
						printf(" ========================================\n\n ");
	
	
						if (aw == "on"){
					    	Qt = (Pd*H_k.transposed()*H_k*Pd);
	
					    	Matrix Qt_det = luinv((H_k*Pd*H_k.transposed() + Rt));
	
					    	Qt = Qt*Qt_det(0,0);
	
							printf("[AW] Next Qt will be: \n(%s)\n\n", Qt.toString().c_str());
	
			    		}
	
	
						H_k = zeros(1,7); //Rows depend on number of trials, columns on number of DoF
						err = zeros(1); //Depend on number of trials
	
	
			    	}else{
			    		var_est = var_est_ant;
			    		printf("Data is NOT suficient to perform an estimation step!\n");
			    		
		    	} 
		    }

		    reading = 1;

		    current_reading = reading;

		    if (reading_t == 60)
		    {

		    	reading_t = 0;
		    	myfile->close();
		    	param_var->close();

		    	H_k = zeros(1,7); //Rows depend on number of trials, columns on number of DoF
				err = zeros(1); //Depend on number of trials
				err_exp = zeros(1);
				Rt = 0.000025*eye(1,1); //Depend on number of trials

	

				Qt = 0.0015*eye(7,7); //Depend on number of DoF

				//Estimation parameters
				u_est = zeros(7);
				//var_est = 0.03*eye(7,7);
				var_est = 0.03*eye(7,7);
				var_est_ant = var_est;

				

				u_bel = zeros(7);
				var_bel = zeros(7,7);

				defineBias(); //reset bias value

				reading = 1; //number of readings between estimation spetps
			    reading_t = 0; //total number of readings provided by controller
			    current_reading = reading; //number of readings provided by controller
						
					 
				std::ostringstream oss;

				oss << "u_est_" << experiment;
				oss << ".txt";
				myfile->open (oss.str().c_str());
				*myfile << u_est.toString().c_str();
				*myfile << "\n";

				if(param == "slow"){
					std::ostringstream iss;

					iss << "param_var_" << experiment;
				 	iss << ".txt";
					param_var->open (iss.str().c_str());
					*param_var << bias.subVector(0,6).toString().c_str();
				 	*param_var << "\n";
				}

				if (experiment == 16){
				 	return false;
				}

				experiment++;

		    }

		}else if (type == "pbatch"){ //pbatch is online estimation with entropy evaluation + data incorporation

			long double det1 = det(var_est);
			long double det2 = det(var_est_ant);

		    long double u = 0.5*log(det2/det1);

			//printf(" \n========================================\n");
			//printf("var_est_ant is: \n(%s)\n", var_est_ant.toString().c_str());
			//printf(" ========================================\n\n ");
//
//			//printf(" \n========================================\n");
//			//printf("var_est is: \n(%s)\n", var_est.toString().c_str());
			//printf(" ========================================\n\n ");



		    printf("u is %.5Le\n", u);
		    if(u >= 0){
		    	printf("Data is suficient to perform an estimation step!\n");		
		        u_est = u_bel + K_gain*(err_exp - err); 
		        var_est_ant = var_est;

		        printf(" \n========================================\n");
				printf("u_est is: \n(%s)\n", u_est.toString().c_str());
				printf(" ========================================\n\n ");
				printf(" \n========================================\n");
				printf("var_est is: \n(%s)\n", var_est.toString().c_str());
				printf(" ========================================\n\n ");


		    	if(reading >= 3){ //Meaning matrixes dimensions were modified
		    		H_k.resize(1,7); //Rows depend on number of trials, columns on number of DoF
					err.resize(1); //Depend on number of trials
					err_exp.resize(1);
					Rt.resize(1,1);
	
					err_exp = zeros(1);
					Rt = 0.000025*eye(1,1); //Depend on number of trials
				}


				H_k = zeros(1,7); //Rows depend on number of trials, columns on number of DoF
				err = zeros(1); //Depend on number of trials


				reading = 1;

		    }else{
		    	var_est = var_est_ant;
		    	printf("Data is NOT suficient to perform an estimation step! Storing information for next measures.\n");

		    	//if (aw == "on"){
			    //	Qt = Pd*H_k.transposed()*H_k*Pd;
			    //	Matrix wtf;
			    //	wtf = luinv(Rt + H_k*Pd*H_k.transposed());
//
//			    //	//Qt = Pd*H_k.transposed()*H_k*Pd*luinv(Rt + H_k*Pd*H_k.transposed());
//				//	printf("[AW] Qt is: \n(%s)\n", Qt.toString().c_str());
//				//	printf("[AW] wtf is: \n(%s)\n", wtf.toString().c_str());
//
		    	//}

		    	H_k.resize(reading,7); //Rows depend on number of trials, columns on number of DoF
				err.resize(reading); //Depend on number of trials
				
				err_exp.resize(reading);
				err_exp = zeros(reading);
				
				Rt.resize(reading,reading);
				Rt = 0.000025*eye(reading,reading); //Depend on number of trials

		    }

		     current_reading = reading;

		}

		
		*myfile << u_est.toString().c_str();
		*myfile << "\n";	

		if(param == "slow"){
		 	
		 	*param_var << bias.subVector(0,6).toString().c_str();
	 		*param_var << "\n";
		 }
	}

}

bool Learning::respond(const Bottle& command, Bottle& reply) {
    yInfo()<<"Got something, echo is on";
    if (command.get(0).asString()=="quit")
        return false;
    else if (command.get(0).asString()=="encoders"){
    	printf("[learning_info] Receiving encoders readings.\n");

    	//retreiving readings information
    	Vector dof(arm->getDOF());
    	Vector offsets(16, 0.0);

    	//u_est[5] = 0.0;
    	offsets.setSubvector(0, u_est);

    	


	   	//printf("torso dof are: \n(%s)\n", command.get(1).asList()->toString().c_str());


    	bool aux;
    	aux = command.get(1).asList()->write(torso_encoders, true);
    	aux = aux && command.get(2).asList()->write(arm_encoders,true);
		printf("Real arm_encoders are: \n(%s)\n\n", arm_encoders.toString().c_str());

		////////////////////////////////////////////////////////////////////////////////////////
		// Vector original_arm_encoders = arm_encoders;
		////////////////////////////////////////////////////////////////////////////////////////

    	arm_encoders = arm_encoders + bias + offsets*(180.0/M_PI);
		printf("Biased arm_encoders are: \n(%s)\n\n", arm_encoders.toString().c_str());


	   	if (!aux){
        	yError()<<"Error getting command info";
        return false;
    	}

    	total_encoders = cat(torso_encoders, arm_encoders);
	   	//printf("dof are: \n(%s)\n", total_encoders.toString().c_str());

	   	dof = total_encoders.subVector(0,9);
		Matrix H = arm->getH(dof * (M_PI/180.0));
		//printf("H is: \n(%s)\n", H.toString().c_str());
    	printf("hand position is: \n(%s)\n\n", arm->EndEffPosition().toString().c_str());


        //End eff postion relative do hand
        finger->getChainJoints(arm_encoders,joints); 

        Vector tipFrame=finger->EndEffPosition((M_PI/180.0)*joints);
        tipFrame.resize(4);
        tipFrame[3] = 1.0; //Necessário para coordenadas homogéneas


        //printf("tip frame is: \n(%s)\n", tipFrame.toString().c_str());

        //End Eff position relative to the world
        Vector left_index_position(4, 0.0);
        left_index_position = H*tipFrame;
    	//printf("tip position is: \n(%s)\n", left_index_position.toString().c_str());

        left_index_position.resize(3);
    	printf("tip position is: \n(%s)\n\n", left_index_position.toString().c_str());


    	////////////////////////////////////////////////////////////////////////////////////////
    	//Next set of functions aim to compute the end-effector position metrics. Comment them for normal operation
  //   	Vector u_est_final(16, 0.0);

  //   	u_est_final[0] =   -0.1219;
  //   	u_est_final[1] =    0.1478; 
  //   	u_est_final[2] =   -0.1658;
  //   	u_est_final[3] =   -0.2901; 
  //   	u_est_final[4] =   -0.0658;
  //   	u_est_final[5] =   -0.2583; 
  //   	u_est_final[6] =    0.1445;


  //   	Vector total_encoders_original = cat(torso_encoders, original_arm_encoders);
	 //   	//printf("dof are: \n(%s)\n", total_encoders.toString().c_str());

	 //   	Vector dof_original = total_encoders_original.subVector(0,9);
		// Matrix H_real = arm->getH(dof_original * (M_PI/180.0));

  //   	Vector left_index_position_real(4, 0.0);
  //   	left_index_position_real = H_real*tipFrame;
  //   	//printf("tip position is: \n(%s)\n", left_index_position.toString().c_str());

  //       left_index_position_real.resize(3);
  //   	printf("tip position (real) is: \n(%s)\n\n", left_index_position_real.toString().c_str());
  //   	Vector aaa = zeros(2);
  //   	aaa[0] = dot(left_index_position_real, n_vector) - distance;
  //   	printf("[Learning_info] Error array is: \n(%s)\n\n", aaa.toString().c_str());

  //   	Vector arm_encoders_final = arm_encoders + u_est_final*(180.0/M_PI);
  //   	Vector total_encoders_final = cat(torso_encoders, arm_encoders_final);
	 //   	//printf("dof are: \n(%s)\n", total_encoders.toString().c_str());

	 //   	Vector dof_final = total_encoders_final.subVector(0,9);
		// Matrix H_final = arm->getH(dof_final * (M_PI/180.0));

		// Vector left_index_position_final(4, 0.0);
		// left_index_position_final = H_final*tipFrame;
  //   	//printf("tip position is: \n(%s)\n", left_index_position.toString().c_str());

  //       left_index_position_final.resize(3);
  //   	printf("tip position (final) is: \n(%s)\n\n", left_index_position_final.toString().c_str());
  //   	aaa[0] = dot(left_index_position_final, n_vector) - distance;
  //   	printf("[Learning_info] Error array is: \n(%s)\n\n", aaa.toString().c_str());

  //   	double cart_err;
  //   	cart_err = sqrt(pow((left_index_position_real[0] - left_index_position[0]),2)+pow((left_index_position_real[1] - left_index_position[1]),2)+pow((left_index_position_real[2] - left_index_position[2]),2));
  //   	printf("[Learning_info] Initial cartesian error value is: \n%f\n\n", cart_err);
    	

  //   	cart_err = sqrt(pow((left_index_position_real[0] - left_index_position_final[0]),2)+pow((left_index_position_real[1] - left_index_position_final[1]),2)+pow((left_index_position_real[2] - left_index_position_final[2]),2));
  //   	printf("[Learning_info] Final cartesian error value is: \n%f\n\n", cart_err);

    	////////////////////////////////////////////////////////////////////////////////////////


    	//Reading error (Expected vs obtained)
        
		err[reading-1] = dot(left_index_position, n_vector) - distance;

        printf("[Learning_info] Error array is: \n(%s)\n\n", err.toString().c_str());
    
        //Obtaining H matrix
        //dof will have incorrect measures; must include torso dof because of GeoJacobian

        Matrix Jacobian=arm->GeoJacobian(dof * (M_PI/180.0));   // iKin works with radians
    	//printf("Arm Jacobian is: \n(%s)\n", Jacobian.toString().c_str());

       	Jacobian = Jacobian.submatrix(0,5,3,9);
    	//printf("Arm Jacobian is: \n(%s)\n", Jacobian.toString().c_str());

        Matrix Jacobian_complete(3,7); //Needed to add finger information
        Vector aux_vec;

        for (int i=0; i<7;i++){ //7 is number of arm dof
        	aux_vec = cross(Jacobian.subcol(3, i, 3), left_index_position) + Jacobian.subcol(0, i, 3);
        	Jacobian_complete.setCol(i, aux_vec);

        }
    	//printf("Complete Jacobian is: \n(%s)\n", Jacobian_complete.toString().c_str());

    	Vector aux_matx = n_vector*Jacobian_complete;

    	H_k.setRow(reading-1, aux_matx);
        //if(type == "batch") H_k.setRow(reading-1, aux_matx);
        //else if(type == "online") H_k.setRow(0, aux_matx);

        printf("[Learning_info] H is \n(%s)\n\n", H_k.toString().c_str());


        reading_t ++;

        //if ( (reading_t > 10) && (reading_t%2 == 0) && param == "slow"){
        if ( (reading_t >= 10) && (reading_t%15 == 0) && param == "slow"){
        //if ( param == "slow"){
        	slowlyVaryingParams();
        }

        printf("[Learning_info] Iteration number: %d\n\n", reading_t);

        if (!askNewSurface()) return false;

        reading ++;
        


    }else if (command.get(0).asString()=="surface"){
    	printf("[controller_info] Receiving surface information.\n");

    	n_vector[0]= command.get(1).asDouble();
    	n_vector[1]= command.get(2).asDouble();
    	n_vector[2]= command.get(3).asDouble();
    	distance = command.get(4).asDouble();

    	printf("[Learning_info] Received obstacle n_vector info: \n(%s)\n", n_vector.toString().c_str());
    	printf("[Learning_info] Received obstacle distance info: \n(%f)\n", distance);

    	startControlFlag = true;

    }else {
        reply.clear();
        reply.addString("error");
    }
    return true;
}

bool Learning::interruptModule() {
	yInfo()<<"Interrupting learning module";
    // inPort.interrupt();
    return true;
}

bool Learning::close() {
	yInfo()<<"closing learning module";
	commandPort.close();
	commandSensorsPort.close();
	myfile->close();

	if(param == "slow"){
		param_var->close();
	}

	return true;
}

bool Learning::defineBias(void){

	//bias[0] = -5.0; //-0.087
	//bias[1] = 7.0; //0.122
	//bias[2] = -10.0; // -0.174
	//bias[3] = 7.0; //0.122
	//bias[4] = 6.0; // 0.105
	//bias[5] = -8.0; //-0.139
	//bias[6] = 9.0; //0.157

	bias[0] = 11.0; //0.1919
	bias[1] = -11.0; //-0.1919
	bias[2] = 7.0; // 0.122
	bias[3] = 17.0; //0.2965
	bias[4] = 7.0; // 0.122
	bias[5] = 17.0; //0.2965
	bias[6] = -7.0; //-0.122

	
	//bias[0] = -17.0; //-0.2965
	//bias[1] = 11.0; //0.1919
	//bias[2] = -22.0; //-0.383
	//bias[3] = 10.0; //0.174
	//bias[4] = 17.0; // 0.2965
	//bias[5] = 17.0; //0.2965
	//bias[6] = -11.0; //-0.1919

	//bias[0] =   11.8736;
	//bias[1] =  -12.9875;
	//bias[2] =    5.5052;
	//bias[3] =   14.7112;
	//bias[4] =    7.7504;
	//bias[5] =   18.2761;
	//bias[6] =   -6.4178;

			 	 		 	


	return true;
}


bool Learning::askNewSurface (void){

	Network yarp;
	
	bool ret = commandSensorsPort.open("/learning/sensor/rpc");
	
	if(!ret) {
	    yError()<<"Cannot open /learning/sensor/rpc port";
	    return false;
	}

	//Connect to left hand index finger sensores
	bool flag = true;    
	while (flag) {
		if (commandSensorsPort.getOutputCount()==0) {
		  printf("Trying to connect to /sensor/rpc \n");
		  yarp.connect("/learning/sensor/rpc", "/sensor/rpc");
		  Time::delay(1);
		} else {
			flag = false;
		}
	}
	Bottle output;
	output.addString("newSurface"); //Tell sensors to retrieve a new surface
	commandSensorsPort.write(output);

	commandSensorsPort.close();

    return true;
}

bool Learning::slowlyVaryingParams (void){

	for(int i = 0; i< 7 ; i++)
	{
		bias[i]+= (double)rand()/(RAND_MAX)*(6)-3;
		//bias[i]+= (double)rand()/(RAND_MAX)*(0.3);
		//bias[i]+= (double)rand()/(RAND_MAX)*(2)-1;
		//printf("%f\t", motor_babbling[i]);
	}


	printf("Offsets will vary by. New true bias is\n: %s \n\n", bias.toString().c_str());
	return true;
}

int main(int argc, char * argv[])
{
    Network yarp;

    Learning module;
    ResourceFinder rf;
    rf.configure(argc, argv);
    
    //rf.setDefaultConfigFile("tutorial_RFModule.ini");
    // rf.setVerbose(true);

    module.runModule(rf);

    yInfo()<<"Main returning...";
    return 0;
}