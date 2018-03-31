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


    /*
    * Interrupt function.
    */
    virtual bool interruptModule();

    virtual bool askNewSurface (void);

    /*
    * Close function, to perform cleanup.
    */
    virtual bool close();

private:

    yarp::os::RpcServer commandPort;
    yarp::os::RpcClient commandSensorsPort;

	double period; //Defines module periodicity   

	std::string entropy; //random or active
	std::string type; //Batch or Online
	std::string aw; //on or off


	iCub::iKin::iCubArm * arm;
	iCub::iKin::iCubFinger * finger;

	iCub::iKin::iKinChain *chain;
	
	//Number of encoders readings 
	int reading;  
	int reading_t;  
	int current_reading;

	bool startControlFlag;


	//Observation and dynamic model parameters
	yarp::sig::Matrix H_k; //Rows depend on number of trials, columns on number of DoF
	Vector err; //Depend on number of trials
	Vector err_exp;
	Matrix Rt; //Depend on number of trials
	Matrix Qt; //Depend on number of DoF
	Matrix Pd;
	Matrix eye_7;

	//Estimation parameters
	Vector u_est;
	Matrix var_est;
	Matrix var_est_ant;
	
	Vector u_bel;
	Matrix var_bel;

	//Obstacle information
	Vector n_vector;
	double distance;

	bool aux;
	Vector arm_encoders;
	Vector torso_encoders;
    Vector total_encoders;
    Vector joints;

    std::ofstream * myfile;

};