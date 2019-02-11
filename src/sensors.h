#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/all.h>

#include <yarp/math/Math.h>

 #include <cmath>



using namespace yarp::os;


class Sensors : public RFModule
{
public:

    Sensors();
    virtual ~Sensors();

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

  	virtual void sendSurface1(void);

    virtual void sendSurface2(void);
  	
    virtual void sendSurface3(void);

    virtual void sendSurface4(void);


private:

    std::string robot; // Uses resorce finder to identify simulation or real system 

	BufferedPort<Bottle> index_finger_port;
	double period; //Defines module periodicity       

	yarp::os::RpcClient commandControllerPort;
	yarp::os::RpcClient commandLearningPort;

	yarp::os::RpcServer commandPort;  // command port

    int surfaceflag; //define surface to send to other modules
    bool sendSurfaceFlag;

};

void sleep( unsigned int unNanosecondsIn );