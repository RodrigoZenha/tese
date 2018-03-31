/*
 * 
 */

#include <string>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

#include <cmath>
#include <vector>
#include <algorithm>

#include <yarp/math/Math.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>

#include <iCub/iKin/iKinFwd.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

//#define DEG2RAD (M_PI/180.0)
//#define RAD2DEG (180.0/M_PI)

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;

class Controller : public yarp::os::RFModule
{
public:


    Controller();
    virtual ~Controller();

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

    /*
    * Close function, to perform cleanup.
    */
    virtual bool close();

    virtual void setDefaultLeftArmPosition (IPositionControl *arm_pos);

    virtual void delWorldEnvironment (void);

    virtual void setWorldEnvironment1 (void);

	virtual void setWorldEnvironment2 (void);
	
	virtual void setWorldEnvironment3 (void);

	virtual void setWorldEnvironment4 (void);

	virtual void checkValidBabbling (void);


private:
    
    std::string robot; // Uses resorce finder to identify simulation or real system 

    double period; //Defines module periodicity

    //std::string modeParam;
    yarp::os::RpcServer commandPort;                    // command port
    yarp::os::RpcClient world_port;  
    //yarp::os::BufferedPort<yarp::os::Bottle> inPort;    // input port
    //yarp::os::BufferedPort<yarp::os::Bottle> outPort;   // output port
    // Set of interfaces we're gonna work with
    IPositionControl *arm_pos;
    IEncoders *arm_encs;
    
    // Left arm encoders
    Vector positions;
    Vector arm_encoders;
    
    // Set of interfaces we're gonna work with
    IEncoders *torso_encs;
    Vector torso_encoders;
    
    
    PolyDriver * robotDevice_arm;
    PolyDriver * robotDevice_torso;
    
    iCub::iKin::iCubArm * arm;
    iCub::iKin::iCubFinger * finger;

    yarp::os::RpcClient commandLearningPort;

    bool stopFlag;
    bool startControlFlag;

    Vector motor_babbling;

    //Obstacle information
	Vector n_vector;
	double distance;
	Vector joints;
	int counter; //used for setDefaultLeftArmPosition 
    int environmentflag; //define environment to create on simulation
    
};

void sleep( unsigned int unNanosecondsIn );

