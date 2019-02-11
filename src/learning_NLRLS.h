#include <stdio.h>
#include <cmath>
#include <fstream>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>


#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>


using namespace yarp::os;
using namespace yarp::math;
using namespace yarp::sig;


class Learning : public RFModule
{
public:

    Learning();
    virtual ~Learning();

    /*
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler and etc.
    */
    virtual bool configure(yarp::os::ResourceFinder &rf);

    /**
     * set the period with which updateModule() should be called
     */
    virtual double getPeriod();

    /*
    * This is our main function. Will be called periodically every getPeriod() seconds.
    */
    virtual bool updateModule();

    /*
    * Message handler. Just echo all received messages.
    */
     virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    virtual bool defineBias(void);


    /*
    * Interrupt function.
    */
    virtual bool interruptModule();

    virtual bool askNewSurface (void);

    virtual bool slowlyVaryingParams (void);

    /*
    * Close function, to perform cleanup.
    */
    virtual bool close();

private:

    yarp::os::RpcServer commandPort;
    yarp::os::RpcClient commandSensorsPort;
     yarp::os::RpcClient commandControllerPort;

	double period; //Defines module periodicity   

	int contacts_number; //Defines number of contacts
    std::string type; //Defines optimization algorithm
    std::string param; //fixed or slow (varying)


	iCub::iKin::iCubArm * arm;
	iCub::iKin::iCubFinger * finger;

	iCub::iKin::iKinChain *chain;
	
	//Number of encoders readings 
	int reading;  
	int experiment;

	bool startControlFlag;


	
	Vector err; //Depend on number of trials

	//Estimation parameters
	Vector u_est;
	Vector u_est_ant;

	//Obstacle information
	Vector n_vector;
	double distance;
	double lambda;
	
	yarp::sig::Matrix H_k; //Rows depend on number of trials, columns on number of DoF
	yarp::sig::Matrix arm_encoders_k;
	yarp::sig::Matrix n_vector_k;
	Vector distance_k;


	Vector arm_encoders;
	Vector torso_encoders;
    Vector total_encoders;
    Vector joints;

    std::ofstream * myfile, * err_file, * param_var;

    Vector bias;

};