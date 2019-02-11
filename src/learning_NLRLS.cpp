#include <learning_NLRLS.h>


Learning::Learning() { }

Learning::~Learning() { }

#define CTRL_DEG2RAD  (M_PI/180.0)


bool Learning::configure(yarp::os::ResourceFinder &rf) {

	Network yarp;

	period = rf.check("period",Value(0.0001)).asDouble();
	type = rf.check("type",Value("GD")).asString().c_str();
    contacts_number = rf.check("contacts",Value(7)).asInt();
    param = rf.check("param",Value("fixed")).asString().c_str();

	printf("Learning Parameters: [Type] %s [Number of contacts] %d \n\n", type.c_str(), contacts_number);
	printf("Estimation Parameters are : %s. \n\n", param.c_str());


	// open all ports
    bool ret = commandPort.open("/learning/rpc");
    ret &= commandSensorsPort.open("/learning/sensor/rpc");
    ret &= commandControllerPort.open("/learning/controller/rpc");

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




    H_k = zeros(contacts_number,7); //Rows depend on number of trials, columns on number of DoF
	err = zeros(contacts_number); //Depend on number of trials
	
	arm_encoders_k = zeros(contacts_number, 10);
	n_vector_k = zeros(contacts_number, 3);

	u_est = zeros(7);

	bias = zeros(16);
	defineBias();

	n_vector.resize(3);

	distance_k.resize(contacts_number);

	lambda = 0.5;

    reading = 0; //total number of readings provided by controller
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

	err_file = new std::ofstream();
	err_file->open ("err_file.txt");

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

	if(reading == contacts_number)
	{	


		bool ret = commandControllerPort.open("/learning/controller/rpc");
		
		if(!ret) {
		    yError()<<"Cannot open /learning/controller/rpc port";
		    return false;
		}

		//Connect to left hand index finger sensores
		bool flag = true;    
		while (flag) {
			if (commandControllerPort.getOutputCount()==0) {
			  printf("Trying to connect to /controller/rpc \n");
			  yarp.connect("/learning/controller/rpc", "/controller/rpc");
			  Time::delay(1);
			} else {
				flag = false;
			}
		}

		Bottle output;
		output.addString("wait"); //Tell controller module to exit
		commandControllerPort.write(output);

		commandControllerPort.close();

		if (type == "GD"){

			u_est_ant = u_est;
			u_est = u_est_ant - lambda*H_k.transposed()*err;

			printf("[Learning_info] u_est is \n(%s)\n\n", u_est.toString().c_str());

			printf(" New Iteration number: \n\n");
			Time::delay(1);

			*err_file << err.toString().c_str();
			*err_file << "\n";
			err_file->close();


			for (int a = 0; a < 30; a ++){ //Number of NL-LS estimation steps (vary if necessary for convergence)

				H_k = zeros(contacts_number,7); //Rows depend on number of trials, columns on number of DoF
				err = zeros(contacts_number); //Depend on number of trials

				//retreiving readings information
		    	Vector dof(arm->getDOF());
		    	Vector offsets(10, 0.0);

		    	offsets.setSubvector(3, u_est);

				printf("offsets are: \n(%s)\n\n", offsets.toString().c_str());
		

				for (int k = 0; k < contacts_number; k++){


					printf("Real arm_encoders are: \n(%s)\n\n", arm_encoders_k.getRow(k).toString().c_str());

					dof = (arm_encoders_k.getRow(k)) + offsets*(180.0/M_PI);
			    	
					printf("Estimated arm_encoders are: \n(%s)\n\n", dof.toString().c_str());


					Matrix H = arm->getH(dof * (M_PI/180.0));
					//printf("H is: \n(%s)\n", H.toString().c_str());
			    	printf("hand position is: \n(%s)\n\n", arm->EndEffPosition().toString().c_str());


			        //End eff postion relative do hand

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



			    	//Reading error (Expected vs obtained)
			        Vector n_vector_k_row = n_vector_k.getRow(k);
					err[k] = dot(left_index_position, n_vector_k_row) - distance_k[k];

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

			    	Vector aux_matx = n_vector_k_row*Jacobian_complete;

			    	H_k.setRow(k, aux_matx);
			        //if(type == "batch") H_k.setRow(reading-1, aux_matx);
			        //else if(type == "online") H_k.setRow(0, aux_matx);

			        printf("[Learning_info] H is \n(%s)\n\n", H_k.toString().c_str());


				}

				u_est_ant = u_est;
				u_est = u_est_ant - lambda*H_k.transposed()*err;

			    printf("[Learning_info] u_est is \n(%s)\n\n", u_est.toString().c_str());

			    *myfile << u_est.toString().c_str();
				*myfile << "\n";	

				printf(" New Iteration number: \n\n");
				Time::delay(1);

			}
		}else if(type == "GN"){

			u_est_ant = u_est;
			u_est = u_est_ant - luinv(H_k.transposed()*H_k)*(H_k.transposed()*err);
			
			printf("[Learning_info] u_est is \n(%s)\n\n", u_est.toString().c_str());

			printf(" New Iteration number: \n\n");
			Time::delay(1);

			*err_file << err.toString().c_str();
			*err_file << "\n";
			err_file->close();


			for (int a = 0; a < 30; a ++){ //Number of NL-LS estimation steps (vary if necessary for convergence)

				H_k = zeros(contacts_number,7); //Rows depend on number of trials, columns on number of DoF
				err = zeros(contacts_number); //Depend on number of trials

				//retreiving readings information
		    	Vector dof(arm->getDOF());
		    	Vector offsets(10, 0.0);

		    	offsets.setSubvector(3, u_est);

				printf("offsets are: \n(%s)\n\n", offsets.toString().c_str());
		

				for (int k = 0; k < contacts_number; k++){


					printf("Real arm_encoders are: \n(%s)\n\n", arm_encoders_k.getRow(k).toString().c_str());

					dof = (arm_encoders_k.getRow(k)) + offsets*(180.0/M_PI);
			    	
					printf("Estimated arm_encoders are: \n(%s)\n\n", dof.toString().c_str());


					Matrix H = arm->getH(dof * (M_PI/180.0));
					//printf("H is: \n(%s)\n", H.toString().c_str());
			    	printf("hand position is: \n(%s)\n\n", arm->EndEffPosition().toString().c_str());


			        //End eff postion relative do hand

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



			    	//Reading error (Expected vs obtained)
			        Vector n_vector_k_row = n_vector_k.getRow(k);
					err[k] = dot(left_index_position, n_vector_k_row) - distance_k[k];

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

			    	Vector aux_matx = n_vector_k_row*Jacobian_complete;

			    	H_k.setRow(k, aux_matx);
			        //if(type == "batch") H_k.setRow(reading-1, aux_matx);
			        //else if(type == "online") H_k.setRow(0, aux_matx);

			        printf("[Learning_info] H is \n(%s)\n\n", H_k.toString().c_str());


				}

				u_est_ant = u_est;
				u_est = u_est_ant - luinv(H_k.transposed()*H_k)*(H_k.transposed()*err);

			    printf("[Learning_info] u_est is \n(%s)\n\n", u_est.toString().c_str());

			    *myfile << u_est.toString().c_str();
				*myfile << "\n";	

				printf(" New Iteration number: \n\n");
				Time::delay(1);


			}
		}

		myfile->close();

		if(param == "slow"){
			param_var->close();
		}

		H_k = zeros(contacts_number,7); //Rows depend on number of trials, columns on number of DoF
		err = zeros(contacts_number); //Depend on number of trials

		defineBias(); //reset bias value
		
		arm_encoders_k = zeros(contacts_number, 10);
		n_vector_k = zeros(contacts_number, 3);
		//u_est = zeros(contacts_number);

		u_est = zeros(7);

		reading = 0; //total number of readings provided by controller

		std::ostringstream oss;
		oss << "u_est_" << experiment;
		oss << ".txt";
		myfile->open (oss.str().c_str());
		*myfile << u_est.toString().c_str();
		*myfile << "\n";

		if(param == "slow"){
			std::ostringstream iss;

			iss << "param_var_" << experiment;
			//kprintf("experiment %s \n", experiment);
		 	iss << ".txt";
			param_var->open (iss.str().c_str());
			*param_var << bias.subVector(0,6).toString().c_str();
		 	*param_var << "\n";
		}

		ret = commandControllerPort.open("/learning/controller/rpc");
		
		if(!ret) {
		    yError()<<"Cannot open /learning/controller/rpc port";
		    return false;
		}

		//Connect to left hand index finger sensores
		flag = true;    
		while (flag) {
			if (commandControllerPort.getOutputCount()==0) {
			  printf("Trying to connect to /controller/rpc \n");
			  yarp.connect("/learning/controller/rpc", "/controller/rpc");
			  Time::delay(1);
			} else {
				flag = false;
			}
		}

		output.clear();
		output.addString("resume"); //Tell controller module to exit
		commandControllerPort.write(output);

		commandControllerPort.close();

		if (experiment == 11){

			bool ret = commandControllerPort.open("/learning/controller/rpc");
		
			if(!ret) {
			    yError()<<"Cannot open /learning/controller/rpc port";
			    return false;
			}

			//Connect to left hand index finger sensores
			bool flag = true;    
			while (flag) {
				if (commandControllerPort.getOutputCount()==0) {
				  printf("Trying to connect to /controller/rpc \n");
				  yarp.connect("/learning/controller/rpc", "/controller/rpc");
				  Time::delay(1);
				} else {
					flag = false;
				}
			}

			Bottle output;
			output.addString("quit"); //Tell controller module to exit
			commandControllerPort.write(output);

			commandControllerPort.close();

			return false; //closes the module
		}

		experiment++;



		

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


	   	if (!aux){
        	yError()<<"Error getting command info";
        	return false;
    	}


		printf("Real arm_encoders are: \n(%s)\n\n", arm_encoders.toString().c_str());

    	arm_encoders = arm_encoders + bias;

    	dof = cat(torso_encoders, arm_encoders.subVector(0,6));

    	arm_encoders_k.setRow(reading, dof);


		printf("arm_encoders_k are: \n(%s)\n\n", arm_encoders_k.getRow(reading).toString().c_str());

    	arm_encoders = arm_encoders + offsets*(180.0/M_PI); //caso offsets tenha valor inicial nulo

    	//arm_encoders = arm_encoders + bias;
		printf("Biased arm_encoders are: \n(%s)\n\n", arm_encoders.toString().c_str());

    	dof = cat(torso_encoders, arm_encoders.subVector(0,6));
	   	//printf("dof are: \n(%s)\n", total_encoders.toString().c_str());

	   	//dof = total_encoders.subVector(0,9);
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



    	//Reading error (Expected vs obtained)
        
		err[reading] = dot(left_index_position, n_vector) - distance;

		n_vector_k.setRow(reading, n_vector);
		distance_k[reading] = distance;


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

    	H_k.setRow(reading, aux_matx);
        //if(type == "batch") H_k.setRow(reading-1, aux_matx);
        //else if(type == "online") H_k.setRow(0, aux_matx);

        printf("[Learning_info] H is \n(%s)\n\n", H_k.toString().c_str());


        printf("[Learning_info] Iteration number: %d\n\n", reading);

        if (!askNewSurface()) return false;

		if(param == "slow"){
		 	

		 	*param_var << bias.subVector(0,6).toString().c_str();
	 		*param_var << "\n";
	        if ( (reading >= 10) && (reading%15 == 0)){
    			slowlyVaryingParams();
        	}
		}

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
	err_file->close();

	if(param == "slow"){
		param_var->close();
	}

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

	return true;

}
			 	 		 	




bool Learning::slowlyVaryingParams (void){

	for(int i = 0; i< 7 ; i++)
	{
		bias[i]+= (double)rand()/(RAND_MAX)*(6)-3;
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