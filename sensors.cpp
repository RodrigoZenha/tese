#include <sensors.h>


Sensors::Sensors() { }

Sensors::~Sensors() { }


bool Sensors::configure(yarp::os::ResourceFinder &rf) {

	Network yarp;

    robot = rf.check("robot",Value("icubSim")).asString().c_str();
	
	period = rf.check("period",Value(0.0001)).asDouble();

	// open all ports
    bool ret = commandControllerPort.open("/sensor/controller/rpc");
	ret &= index_finger_port.open("/indexSkinRead");
	ret &= commandLearningPort.open("/sensor/learning/rpc");

    ret &= commandPort.open("/sensor/rpc");

    if(!ret) {
        yError()<<"Cannot open some of the ports";
        return false;
    }


    if(!attach(commandPort)) { // Connect to respond method
        yError()<<"Cannot attach to the commandPort";
        return false;
    }


    //Connect to left hand index finger sensores
	bool flag = true;    
    while (flag) {
	    if (index_finger_port.getInputCount()==0) {
	      printf("Trying to connect to /icubSim/skin/left_hand_comp \n");
	      yarp.connect("/icubSim/skin/left_hand_comp", "/indexSkinRead");
	      Time::delay(1);
	    } else {
	    	flag = false;
	    }
	}

	//Sending surface information to controller. Later this shall be obtained through vision
	flag = true;    
    while (flag) {
	    if (commandControllerPort.getOutputCount()==0) {
	      printf("Trying to connect to /controller/rpc \n");
	      yarp.connect("/sensor/controller/rpc", "/controller/rpc");
	      Time::delay(1);

	    } else {
	    	flag = false;
	    }
	}

	//Sending surface information to learning module. Later this shall be obtained through vision
	flag = true;    
    while (flag) {
	    if (commandLearningPort.getOutputCount()==0) {
	      printf("Trying to connect to /learning/rpc \n");
	      yarp.connect("/sensor/learning/rpc", "/learning/rpc");
	      Time::delay(1);

	    } else {
	    	flag = false;
	    }
	}

	sendSurface1();
	sendSurfaceFlag = false;


}

double Sensors::getPeriod() {
    return period; // module periodicity (seconds)
}

bool Sensors::updateModule() {

		Network yarp;


        Bottle *input = index_finger_port.read();
        if (input!=NULL) {
            //printf("[Sensors_info] Left hand sensors got %f\n ", input->get(0).asDouble());

        }
            
        if (input->get(0).asDouble() != 0.0){

        	printf("[sensors_info] Contact detected at index finger\n");

        	bool flag = true;    
		    while (flag) {
			    if (commandControllerPort.getOutputCount()==0) {
			      printf("Trying to connect to /controller/rpc \n");
			      yarp.connect("/sensor/controller/rpc", "/controller/rpc");
			      Time::delay(1);

			    } else {
			    	flag = false;
			    }
			}

            Bottle output;
            output.addString("stop");
            printf("[sensors_info] Sending \"stop\" signal to controller module.\n\n");
            commandControllerPort.write(output);

            Time::delay(13);


        }

    return true;

}

 bool Sensors::respond(const Bottle& command, Bottle& reply) {
 	yInfo()<<"Got something, echo is on";
    if (command.get(0).asString()=="quit")
        return false;
    else if (command.get(0).asString()=="newSurface"){
    	
    	if (robot == "icubSim"){

    		sendSurfaceFlag = true; //Necessário pois respond é assincrono e envia surface antes de esperar que controller termine
    		printf("Will be sending new surface to control and learning modules\n");

    		if (sendSurfaceFlag == true){ //Necessário pois respond é assincrono e envia surface antes de esperar que controller termine
	            if (surfaceflag == 0){		//sendSufaceFlag só é true quando se trata de icubSim. Desnecessário voltar a confirmar
					sendSurface1();
				
				}else if(surfaceflag == 1){
					sendSurface2();

				}else if(surfaceflag == 2){
					sendSurface3();

				}else{
					sendSurface4();

				}

				sendSurfaceFlag = false;
			}

			
    	}

    }else {
        reply.clear();
        reply.addString("error");
    }

	return true;
}


bool Sensors::interruptModule() {
	yInfo()<<"Interrupting sensors module";
    // inPort.interrupt();
    return true;
}

bool Sensors::close() {
	yInfo()<<"closing sensores module";

	commandControllerPort.close();
	index_finger_port.close();
	commandLearningPort.close();
	commandPort.close();

	return true;
}

void sleep( unsigned int unNanosecondsIn )
{
	clock_t wait_nanoseconds = (clock_t) unNanosecondsIn;
	clock_t start_time = clock();
	while( clock() != start_time + wait_nanoseconds );
	return;
}

void Sensors::sendSurface1 (void){

	Bottle output;
    output.addString("surface");
    output.addDouble(sqrt(2)/2);
    output.addDouble(0);
    output.addDouble(sqrt(2)/2);
    //double distance = (-sqrt(2)/5);
    double distance = (-sqrt(2)/8);
    //double distance = 0.6-0.5976;
    output.addDouble(distance);
    printf("[sensors_info] Sending \"surface\" signal to controller module.\n\n");
    commandControllerPort.write(output);
    printf("[sensors_info] Sending \"surface\" signal to learning module.\n\n");
    commandLearningPort.write(output);

    surfaceflag = 0;

    return;
}

void Sensors::sendSurface2 (void){


	Bottle output;
	output.addString("surface");
	output.addDouble(sqrt(2)/2);
	output.addDouble(sqrt(2)/2);
	output.addDouble(0);
	//double distance = (-sqrt(2)/5);
	double distance = (-3*sqrt(2)/10);
	//double distance = 0.6-0.5976;
	output.addDouble(distance);
	printf("[sensors_info] Sending \"surface\" signal to controller module.\n\n");
	commandControllerPort.write(output);
	printf("[sensors_info] Sending \"surface\" signal to learning module.\n\n");
	commandLearningPort.write(output);

    surfaceflag = 3;

    return;
}

void Sensors::sendSurface3 (void){


	Bottle output;
	output.addString("surface");
	output.addDouble(1);
	output.addDouble(0);
	output.addDouble(0);
	//double distance = (-sqrt(2)/5);
	double distance = (-0.4);
	//double distance = 0.6-0.5976;
	output.addDouble(distance);
	printf("[sensors_info] Sending \"surface\" signal to controller module.\n\n");
	commandControllerPort.write(output);
	printf("[sensors_info] Sending \"surface\" signal to learning module.\n\n");
	commandLearningPort.write(output);

    surfaceflag = 3;

    return;
}

void Sensors::sendSurface4 (void){


	Bottle output;
	output.addString("surface");
	output.addDouble(0);
	output.addDouble(-1);
	output.addDouble(0);
	//double distance = (-sqrt(2)/5);
	double distance = (-0.05);
	//double distance = 0.6-0.5976;
	output.addDouble(distance);
	printf("[sensors_info] Sending \"surface\" signal to controller module.\n\n");
	commandControllerPort.write(output);
	printf("[sensors_info] Sending \"surface\" signal to learning module.\n\n");
	commandLearningPort.write(output);

    surfaceflag = 0;

    return;
}


int main(int argc, char * argv[])
{
    Network yarp;

    Sensors module;
    ResourceFinder rf;
    rf.configure(argc, argv);
    
    //rf.setDefaultConfigFile("tutorial_RFModule.ini");
    // rf.setVerbose(true);

    module.runModule(rf);

    yInfo()<<"Main returning...";
    return 0;
}