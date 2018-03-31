//#include <cmath>
//#include <vector>
//#include <algorithm>
//
//#include <yarp/math/Math.h>
#include <controller.h>
//#include <yarp/os/LogStream.h>
//#include <yarp/dev/PolyDriver.h>
//#include <yarp/dev/ControlBoardInterfaces.h>
//#include <yarp/sig/Vector.h>
//
//#include <iCub/iKin/iKinFwd.h>
//
////#define DEG2RAD (M_PI/180.0)
////#define RAD2DEG (180.0/M_PI)
//
//using namespace yarp::os;
//using namespace yarp::dev;
//using namespace yarp::sig;
//using namespace yarp::math;
//using namespace iCub::iKin;

const double CTRL_DEG2RAD = M_PI/180.0;

Controller::Controller() { }

Controller::~Controller() { }


bool Controller::configure(yarp::os::ResourceFinder &rf) {

    Network yarp;
    yInfo()<<"Configuring the controller module...";

    RpcClient right_arm_port;

    stopFlag = false;
    startControlFlag = false;
    environmentflag = 0; //define environment to create on simulation


    n_vector.resize(3); //3D scenario


    robot = rf.check("robot",Value("icubSim")).asString().c_str();

    period = rf.check("period",Value(0.0001)).asDouble();

 
    // open all ports
    bool ret = commandPort.open("/controller/rpc");
    /*
    * ret &= inPort.open("/controller/in");
    * ret &= outPort.open("/controller/out");
    */
    if (robot == "icubSim") ret &=world_port.open("/local/world");
    ret &=right_arm_port.open("/local/right_arm");
    ret &= commandLearningPort.open("/controller/learning/rpc");

	if(!ret) {
        yError()<<"Cannot open some of the ports";
        return false;
    }

    
    // Set right arm initial position
    bool flag = true;
    while (flag) {
	    if (right_arm_port.getOutputCount()==0) {
	      printf("Trying to connect to /%s/right_arm/rpc:i \n", robot.c_str());
	      yarp.connect("/local/right_arm","/"+robot+"/right_arm/rpc:i");
	      //yarp.connect("/local/right_arm",robot+"/right_arm/rpc:i");
	      Time::delay(1);
	    } else {
	    	flag = false;
	    }
	}

	flag = true;
    if (robot == "icubSim"){
	    while (flag) {
		    if (world_port.getOutputCount()==0) {
		      printf("Trying to connect to /icubSim/world \n");
		      yarp.connect("/local/world","/icubSim/world");
		      Time::delay(1);
		    } else {
		    	flag = false;
		    }
		}
	}


	Bottle command; 
	Bottle cmd;
	Bottle response;

	cmd.clear();
    cmd.addString("set");
    cmd.addString("poss");
    
    command.addDouble(-78.7);
    command.addDouble(75.5);
    command.addDouble(0.0);
    command.addDouble(49.9);
    command.addDouble(0.0);
    command.addDouble(0.0);
    command.addDouble(0.0);
    command.addDouble(59.0);
    command.addDouble(20.0);
    command.addDouble(20.0);
    command.addDouble(20.0);
    command.addDouble(10.0);
    command.addDouble(10.0);
    command.addDouble(10.0);
    command.addDouble(10.0);
    command.addDouble(10.0);

    cmd.addList() = command;

	printf("[rpc] Sending command to iCub right arm... %s\n", cmd.toString().c_str());
	response.clear();
	right_arm_port.write(cmd,response);
	printf("[rpc] Got response: %s\n\n", response.toString().c_str());


    if(!attach(commandPort)) { // Connect to respond method
        yError()<<"Cannot attach to the commandPort";
        return false;
    }


    // Initializing motor interface for left arm
	Property options;
	options.put("device", "remote_controlboard");
	options.put("local", "/local/left_arm");                 //local port names
	options.put("remote", "/"+robot+"/left_arm");         //where we connect to

	// Create the driver we use the PolyDriver
	robotDevice_arm = new PolyDriver (options);
	// PolyDriver robotDevice_arm(options);
	if (!robotDevice_arm->isValid()) {
	    printf("Device not available.  Here are the known devices:\n");
	    printf("%s", Drivers::factory().toString().c_str());
	    return 1;
	}

	bool ok;
	ok = robotDevice_arm->view(arm_pos);
	ok = ok && robotDevice_arm->view(arm_encs);
	if (!ok) {
	    printf("Problems acquiring interfaces\n");
	    return 0;
	}

	int nj_arm=0;
    arm_pos->getAxes(&nj_arm);
    Vector tmp;
    arm_encoders.resize(nj_arm);
    tmp.resize(nj_arm);
	positions.resize(nj_arm);
	positions = zeros(16);
	torso_encoders.resize(3);

	// Set reference accelerations
	int i;
    for (i = 0; i < nj_arm; i++) {
        tmp[i] = 50.0;
	}
    arm_pos->setRefAccelerations(tmp.data());
 
    //Set reference speeds
    for (i = 0; i < nj_arm; i++) {
        tmp[i] = 30.0;
        // pos->setRefSpeed(i, tmp[i]);
    }
 
	arm_pos->setRefSpeeds(tmp.data());

	counter = 5; //used in setDefaultLeftArmPosition
	setDefaultLeftArmPosition(arm_pos);

	
	// Setting environment in order to get Jacobian
	
	arm = new iCub::iKin::iCubArm ("left");
	//iCub::iKin::iCubArm arm("left");   // declare the arm object
	arm->setAllConstraints(false);   // we don't need to update the limits from the robot to compute the Jacobian
	arm->releaseLink(0);   // release torso pitch joint
	arm->releaseLink(1);   // release torso roll joint
	arm->releaseLink(2);   // release torso yaw joint

	finger = new iCub::iKin::iCubFinger ("left_index"); //Necessary later

	// Initializing motor interface for torso
	Property options_2;
	options_2.put("device", "remote_controlboard");
	options_2.put("local", "/local/torso");                 //local port names
	options_2.put("remote", "/"+robot+"/torso");         //where we connect to

	// Create the driver we use the PolyDriver
	
	robotDevice_torso = new PolyDriver (options_2);
	//PolyDriver robotDevice_torso(options_2);
	if (!robotDevice_torso->isValid()) {
	    printf("Device not available.  Here are the known devices:\n");
	    printf("%s", Drivers::factory().toString().c_str());
	    return 1;
	}


	ok =robotDevice_torso->view(torso_encs);
	
	if (!ok) {
	    printf("Problems acquiring interfaces\n");
	    return 0;
	}
    

	if (robot == "icubSim"){

		delWorldEnvironment();

		//printf("%d\n", environmentflag);
		
		if (environmentflag == 0){
			setWorldEnvironment1();
		
		}else if(environmentflag == 1){
			setWorldEnvironment2();

		}else if(environmentflag == 2){
			setWorldEnvironment3();

		}else{
			setWorldEnvironment4();

		}

		//printf("%d oi\n", environmentflag);
    }


	
	srand( unsigned(time(NULL)));
	motor_babbling = zeros(10);
	right_arm_port.close();
    // everything is fine
    return true;
}


double Controller::getPeriod() {
    return period; // module periodicity (seconds)
}


bool Controller::updateModule() {

	Network yarp;

	 // random vector for motor babling, from waist to wrist motors
	
	//printf("%d\n", environmentflag);

	// End effector velocity during control
	Vector pos_vel(6, 0.0);

	// fill in the vector of degrees of freedom
	Vector dofs(arm->getDOF());

	if (startControlFlag == false){

		printf("No surface detect by sensors \n");
		return true;
	}


	//Check if stopFlag is active and send encoders readings to learning module
	if (stopFlag == true){

		arm_pos->stop();
		startControlFlag = false;
		Time::delay(10);
		//sleep(10000000); // wait 10 sec

		//Sending encoders readings to learning module
		bool flag = true;
		while (flag) {
		    if (commandLearningPort.getOutputCount()==0) {
		      printf("Trying to connect to /learning/rpc \n");
		      yarp.connect("/controller/learning/rpc","/learning/rpc");
		      Time::delay(1);
		    } else {
		    	flag = false;
		    }
		}

		Bottle output;
        output.addString("encoders");
        printf("[sensors_info] Sending \"encoders\" signal to learning module.\n\n");

        arm_encs->getEncoders(arm_encoders.data());
		printf("arm_encoders readings are: \n(%s)\n\n", arm_encoders.toString().c_str());

        torso_encs->getEncoders(torso_encoders.data());
        
        //output.addDouble(torso_encoders[2]);
        //output.addDouble(torso_encoders[1]);
        //output.addDouble(torso_encoders[0]);
        //output.addDouble(arm_encoders[0]);
        //output.addDouble(arm_encoders[1]);
        //output.addDouble(arm_encoders[2]);
        //output.addDouble(arm_encoders[3]);
        //output.addDouble(arm_encoders[4]);
        //output.addDouble(arm_encoders[5]);
        //output.addDouble(arm_encoders[6]);
        output.addList().read(torso_encoders, true);
        output.addList().read(arm_encoders, true);

        //Vector v2;
        //output.get(1).asList()->write(v2);
  		//printf("Vector is %s\n", v2.toString().c_str());

        commandLearningPort.write(output);
    	
        motor_babbling = zeros(10);
		delWorldEnvironment();

		setDefaultLeftArmPosition(arm_pos);

		if (robot == "icubSim"){


			//printf("%d\n", environmentflag);
			
			if (environmentflag == 0){
				setWorldEnvironment1();
			
			}else if(environmentflag == 1){
				setWorldEnvironment2();

			}else if(environmentflag == 2){
				setWorldEnvironment3();

			}else{
				setWorldEnvironment4();

			}

			//printf("%d oi\n", environmentflag);
    	}

		//srand( unsigned(time(NULL)));

		stopFlag = false;

	}

    

	

	// retrieve here joints from the torso and the left_arm
    arm_encs->getEncoders(arm_encoders.data());	
    
    torso_encs->getEncoders(torso_encoders.data());
 

	
	dofs[0]=torso_encoders[2];   // be careful of the order here
	dofs[1]=torso_encoders[1];
	dofs[2]=torso_encoders[0];
	dofs[3]=arm_encoders[0];
	dofs[4]=arm_encoders[1];
	dofs[5]=arm_encoders[2];
	dofs[6]=arm_encoders[3];
	dofs[7]=arm_encoders[4];
	dofs[8]=arm_encoders[5];
	dofs[9]=arm_encoders[6];

	// Generate random motor babling, from waist to wrist motors
	motor_babbling.resize(7);


	for(int i = 0; i< 7 ; i++)
	{
		motor_babbling[i]+= (double)rand()/(RAND_MAX)*(0.4)-0.2;
		
		if (motor_babbling[i]>1.2){

			motor_babbling[i] = 1.2;

		}else if(motor_babbling[i]< -1.2){

			motor_babbling[i] = -1.2;
		}
		//printf("%f\t", motor_babbling[i]);
	}
	//printf("\n");

	checkValidBabbling();

	yarp::sig::Matrix Jacobian=arm->GeoJacobian(dofs * (M_PI/180.0));   // iKin works with radians

	//Remove dof relative to torso
	Matrix Jacobian2(3,7);

	Jacobian2= Jacobian.submatrix(0,5,3,9);

	finger->getChainJoints(arm_encoders,joints);
	
    Vector tipFrame=finger->EndEffPosition((M_PI/180.0)*joints); //Finger to hand transform
	
    //printf("[controller info] tipFrame is: \n(%s)\n\n", tipFrame.toString().c_str());


    Vector v_hand, w_hand; 

	pos_vel = Jacobian2*motor_babbling; //Velocity reative to left hand
	v_hand = pos_vel.subVector(0,2);
	w_hand = pos_vel.subVector(3,5);

	pos_vel = v_hand + cross(w_hand, tipFrame); //Velocity reative to index finger


	//printf("[Control_info] Estimated end-effector velocity:\n");
	//for (int i= 0; i < 6; i++)
	//	printf("%f\t", pos_vel[i]);
	//printf("\n");

	pos_vel.resize(3);

	double dot_prod = dot(n_vector, pos_vel); 	


	if(dot_prod <= 0.0){

		arm_encoders[0]+=motor_babbling[0];
		arm_encoders[1]+=motor_babbling[1];
		arm_encoders[2]+=motor_babbling[2];
		arm_encoders[3]+=motor_babbling[3];
		arm_encoders[4]+=motor_babbling[4];
		arm_encoders[5]+=motor_babbling[5];
		arm_encoders[6]+=motor_babbling[6];

		arm_pos->positionMove(arm_encoders.data());

		bool done = false;
		while (!done) {
			if(stopFlag == true) done = true;
		   arm_pos->checkMotionDone(&done);
		   Time::delay(0.001);   // or any suitable delay

		}

	}

    return true;
}


bool Controller::respond(const Bottle& command, Bottle& reply) {
    yInfo()<<"Got something, echo is on";
    if (command.get(0).asString()=="quit")
        return false;
    else if (command.get(0).asString()=="stop"){
    	printf("[controller_info] Received a stop signal.\n");

    	
    	stopFlag = true;

    }else if (command.get(0).asString()=="surface"){


    	printf("[controller_info] Receiving surface information.\n");

    	n_vector[0]= command.get(1).asDouble();
    	n_vector[1]= command.get(2).asDouble();
    	n_vector[2]= command.get(3).asDouble();
    	distance = command.get(4).asDouble();


    	printf("[Control_info] Received obstacle n_vector info: \n(%s)\n", n_vector.toString().c_str());
    	printf("[Control_info] Received obstacle distance info: \n(%f)\n", distance);


    	startControlFlag = true;

    }else {
        reply.clear();
        reply.addString("error");
    }
    return true;
}


bool Controller::interruptModule() {
    yInfo()<<"Interrupting controller module";
    // inPort.interrupt();
    return true;
}


bool Controller::close() {
    yInfo()<<"closing controller module";

	delWorldEnvironment();

    commandPort.close();
    world_port.close();
    robotDevice_arm->close();
    robotDevice_torso->close();
    //robotDevice_torso.close();*/
    
    //inPort.close();
    
    // you can force writing remaining data
    // using outPort.writeStrict();
    
    //outPort.close();
    
    return true;
}

void sleep( unsigned int unNanosecondsIn )
{
	clock_t wait_nanoseconds = (clock_t) unNanosecondsIn;
	clock_t start_time = clock();
	while( clock() != start_time + wait_nanoseconds );
	return;
}

void Controller::setDefaultLeftArmPosition (IPositionControl *arm_pos){


	//positions[0]=-94.0;
//	//positions[1] = counter * 10.0 ; //Necessário counter pois o valor de positions varia com o mator babling
//	//if (positions[1] > 75.0) {
//	//	positions[1] = 10;
//	//	counter = 1;
//	//}
//	//positions[2]=79.6;
//	//positions[3]=40.0;
//	//positions[4]=32.4;
//	//positions[5]=0.0;
//	//positions[6]=0.0;
//	//positions[7]=59.0;
//	//positions[8]=20.0;
//	//positions[9]=20.0;
//	//positions[10]=180.0;
//	//positions[11]=50.0;
//	//positions[12]=0.0;
//	//positions[13]=90.0;
//	//positions[14]=180.0; 
//	//positions[15]=270.0;
//	//arm_pos->positionMove(positions.data());
//
//	//bool done=false;
//
	//while(!done)
    // {
    //     arm_pos->checkMotionDone(&done);
    //     Time::delay(0.1);
  	//}

  	positions[0]=-50.0;
	positions[1] = counter * 10.0 ; //Necessário counter pois o valor de positions varia com o mator babling
	if (positions[1] > 100.0) {
		
		counter = 5;
		positions[1] = counter*10;
	}
	positions[2]=54.9;
	positions[3]=105.8;
	positions[4]=pow(-1,counter)*80;
	positions[5]=-23.4;
	positions[6]=10.8;
	positions[7]=59.0;
	positions[8]=20.0;
	positions[9]=20.0;
	positions[10]=180.0;
	positions[11]=50.0;
	positions[12]=0.0;
	positions[13]=90.0;
	positions[14]=180.0; 
	positions[15]=270.0;
	arm_pos->positionMove(positions.data());

	printf("\n [rpc] Sending command to iCub left arm... %s\n", positions.toString().c_str());

	bool done=false;

	while(!done)
     {
         arm_pos->checkMotionDone(&done);
         Time::delay(0.1);
  	}

	counter++;

	return;
}

void Controller::delWorldEnvironment (void){

	Bottle cmd, response;
	
	 // Delete world Environment
	cmd.clear();
    cmd.addString("world");
    cmd.addString("del");
    cmd.addString("all");
    

	printf("[rpc] Sending command to world... %s\n", cmd.toString().c_str());
	response.clear();
	world_port.write(cmd,response);
	printf("[rpc] Got response: %s\n\n", response.toString().c_str());

	return;
	
}

void Controller::setWorldEnvironment1 (void){

	Bottle cmd, response;

	
	 // Set world Environment
	cmd.clear();
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("sbox");
    cmd.addInt(1);
    cmd.addDouble(0.0001);
    cmd.addDouble(0.7);
    cmd.addInt(0);
    //cmd.addDouble(0.5976);
    cmd.addDouble(0.7476);
    cmd.addDouble(0.374); //0.4 - 0.026
    cmd.addInt(1);
    cmd.addInt(0);
    cmd.addInt(0);
    //cmd.addString("world");
    //cmd.addString("mk");
    //cmd.addString("sbox");
    //cmd.addInt(1);
    //cmd.addDouble(1);
    //cmd.addDouble(0.1);
    //cmd.addInt(0);
    //cmd.addDouble(0.5);
    //cmd.addDouble(0.4);
    //cmd.addInt(1);
    //cmd.addInt(0);
    //cmd.addInt(0);


	printf("[rpc] Sending command to world... %s\n", cmd.toString().c_str());
	response.clear();
	world_port.write(cmd,response);
	printf("[rpc] Got response: %s\n\n", response.toString().c_str());
	
	cmd.clear();
	response.clear();
	
	cmd.addString("world");
    cmd.addString("rot");
    cmd.addString("sbox");
    cmd.addInt(1);
    cmd.addInt(-45);
    cmd.addInt(0);
    cmd.addInt(0);

    printf("[rpc] Sending command to world... %s\n", cmd.toString().c_str());
	world_port.write(cmd,response);
	printf("[rpc] Got response: %s\n\n", response.toString().c_str());

	environmentflag = 0;

	// world mk sbox 1 0.001 0.7 0 0.7476 0.374 1 0 0
	//world rot sbox 1 -45 0 0

	return;
}

void Controller::setWorldEnvironment2 (void){
	
	Bottle cmd, response;

    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("sbox");
    cmd.addDouble(0.0001);
    cmd.addInt(1);
    cmd.addDouble(1.2);
    cmd.addDouble(0.25);
    cmd.addDouble(0.7476);
    cmd.addDouble(0.324);
    cmd.addInt(1);
    cmd.addInt(0);
    cmd.addInt(0);



	printf("[rpc] Sending command to world... %s\n", cmd.toString().c_str());
	world_port.write(cmd,response);
	printf("[rpc] Got response: %s\n\n", response.toString().c_str());
	
	cmd.clear();
	response.clear();
	
	cmd.addString("world");
    cmd.addString("rot");
    cmd.addString("sbox");
    cmd.addInt(1);
    cmd.addInt(0);
    cmd.addInt(-45);
    cmd.addInt(0);

    printf("[rpc] Sending command to world... %s\n", cmd.toString().c_str());
	world_port.write(cmd,response);
	printf("[rpc] Got response: %s\n\n", response.toString().c_str());



	environmentflag = 3;

	// world mk sbox 0.001 1 1.2 0.25 0.7476 0.324 1 0 0
	//world rot sbox 1 0 -45 0


	return;

}

void Controller::setWorldEnvironment3 (void){
	
	Bottle cmd, response;

    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("sbox");
    cmd.addDouble(1.2);
    cmd.addInt(1);
    cmd.addDouble(0.0001);
    cmd.addDouble(0.0);
    cmd.addDouble(0.7476);
    cmd.addDouble(0.374); //0.45-0.026
    cmd.addInt(1);
    cmd.addInt(0);
    cmd.addInt(0);



	printf("[rpc] Sending command to world... %s\n", cmd.toString().c_str());
	world_port.write(cmd,response);
	printf("[rpc] Got response: %s\n\n", response.toString().c_str());
	
	

	environmentflag = 3;

	// world mk sbox 1.2 1 0.001 0.0 0.7476 0.374 1 0 0


	return;

}

void Controller::setWorldEnvironment4 (void){
	
	Bottle cmd, response;

    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("sbox");
    cmd.addDouble(0.0001);
    cmd.addInt(1);
    cmd.addDouble(0.9);
    cmd.addDouble(-0.05);
    cmd.addDouble(0.6);
    cmd.addDouble(0.5);
    cmd.addInt(1);
    cmd.addInt(0);
    cmd.addInt(0);



	printf("[rpc] Sending command to world... %s\n", cmd.toString().c_str());
	world_port.write(cmd,response);
	printf("[rpc] Got response: %s\n\n", response.toString().c_str());
		

	environmentflag = 0;

	// world mk sbox 0.001 1 0.9 -0.05 0.6 0.5 1 0 0


	return;

}



void Controller::checkValidBabbling (void)
{


	motor_babbling[0]=((motor_babbling[0] + arm_encoders[0])<-94.0) ? 0 : (((motor_babbling[0] + arm_encoders[0])>9.4) ? 0 : motor_babbling[0]);

	motor_babbling[1]=((motor_babbling[1] + arm_encoders[1])<0) ? 0 : (((motor_babbling[1] + arm_encoders[1])>160.8) ? 0 : motor_babbling[1]);

	motor_babbling[2]=((motor_babbling[2] + arm_encoders[2])<-36.2) ? 0 : (((motor_babbling[2] + arm_encoders[2])>79.5) ? 0 : motor_babbling[2]);

	motor_babbling[3]=((motor_babbling[3] + arm_encoders[3])<17.5) ? 0 : (((motor_babbling[3] + arm_encoders[3])>105.8) ? 0 : motor_babbling[3]);

	motor_babbling[4]=((motor_babbling[4] + arm_encoders[4])<-90.0) ? 0 : (((motor_babbling[4] + arm_encoders[4])>90.0) ? 0 : motor_babbling[4]);

	motor_babbling[5]=((motor_babbling[5] + arm_encoders[5])<-90.0) ? 0 : (((motor_babbling[5] + arm_encoders[5])>0.0) ? 0 : motor_babbling[5]);

	motor_babbling[6]=((motor_babbling[6] + arm_encoders[6])<-19.5) ? 0 : (((motor_babbling[6] + arm_encoders[6])>39.5) ? 0 : motor_babbling[6]);

	return;

}	



int main(int argc, char * argv[])
{
    Network yarp;

    Controller module;
    ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setDefaultContext("v1");
    //rf.setDefaultConfigFile("tutorial_RFModule.ini");
    // rf.setVerbose(true);

    module.runModule(rf);

    yInfo()<<"Main returning...";
    return 0;
}