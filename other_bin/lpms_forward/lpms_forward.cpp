#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#include <cstdio>
#include <thread>
#include <functional>
#include <chrono>

#include <LpmsSensorI.h>
#include <LpmsSensorManagerI.h>

#include <iit/advr/zmq_publisher.h>
#include <protobuf/quaternion.pb.h>

#ifdef __XENO_PIPE__
static const std::string __pipe_prefix ( "/proc/xenomai/registry/rtipc/xddp/" );
#else
static const std::string __pipe_prefix ( "/tmp/" );
#endif

extern void main_common ( int *argcp, char *const **argvp, __sighandler_t sig_handler );
extern void set_main_sched_policy ( int );

static int main_loop = 1;

static void shutdown ( int sig __attribute__ ( ( unused ) ) ) {
    main_loop = 0;
    printf ( "got signal .... Shutdown\n" );
}

//typedef void (*LpmsCallback)(ImuData d, const char* id);
std::function<void ( const ImuData &,const char * ) > lpms_cb;
static void cb_hook ( ImuData imu_data, const char * id ) {
    lpms_cb ( imu_data,id );
}

using namespace std::chrono;
#define SLEEP(t)	std::this_thread::sleep_for(t);

/////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////
namespace {

class ImuHandler {
    typedef std::map<std::string, std::string> jmap_t;
#define JPDO(x) jpdo[#x] = std::to_string(x);

    typedef struct ImuDataExt {
        ImuDataExt() {}
        ImuDataExt ( const ImuData &d ) {
            data=d;
            qW=data.q[0];
            qX=data.q[1];
            qY=data.q[2];
            qZ=data.q[3];
        }
        int sprint ( char *buff, size_t size ) {
            return snprintf ( buff, size, "%f\t%f\t%f\t%f\t%f", data.timeStamp, data.q[0], data.q[1], data.q[2], data.q[3] );
        }
        void to_map ( jmap_t & jpdo ) {
            JPDO ( qW );
            JPDO ( qX );
            JPDO ( qY );
            JPDO ( qZ );
        }
        void pb_toString ( std::string * pb_str ) {
            gazebo::msgs::Quaternion quat;
            quat.set_x ( qX );
            quat.set_y ( qY );
            quat.set_z ( qZ );
            quat.set_w ( qW );
            quat.SerializeToString ( pb_str );
        }
    private :
        ImuData data;
        float qW,qX,qY,qZ;
    } ImuDataExt;
    typedef Publisher<ImuDataExt> ImuPub;

public:
    ~ImuHandler();
    void setup ( std::string pipe_name, bool use_cb );
    void operator() () const;
    void imu_data_cb ( const ImuData &, const char* );
    void stop() {
        loop = false;
    }
private:
    int			xddp_sock;
    LpmsSensorManagerI*	manager;
    LpmsSensorI* 	lpms;
    bool 		loop;
    ImuPub		* imuPub;
};


ImuHandler::~ImuHandler() {
    delete imuPub;
    // Removes the initialized sensor
    manager->removeSensor ( lpms );
    // Deletes LpmsSensorManager object
    delete manager;
}



void ImuHandler::imu_data_cb ( const ImuData &imu_data, const char* id ) {

    printf ( "Fps %f\t", lpms->getFps() );
    printf ( "<ImuHandler method> [%s] Timestamp=%f, qW=%f, qX=%f, qY=%f, qZ=%f\n",
             id, imu_data.timeStamp, imu_data.q[0], imu_data.q[1], imu_data.q[2], imu_data.q[3] );

    ImuDataExt zzz ( imu_data );
    imuPub->publish ( zzz );

    int nbytes;
    if ( xddp_sock > 0 ) {
        nbytes = write ( xddp_sock, ( void* ) &imu_data, sizeof ( imu_data ) );
    }

}

void ImuHandler::setup ( std::string pipe_name, bool use_cb ) {

    // Gets a LpmsSensorManager instance
    manager = LpmsSensorManagerFactory();

    manager->startListDevices ( false );
    while ( manager->listDevicesBusy() ) {
        printf ( "." );
        SLEEP ( seconds ( 1 ) );
    }
    LpmsDeviceList dev_list = manager->getDeviceList();
    //printf("---> %d\n",dev_list.nDevices);
    for ( int i=0; i<dev_list.nDevices; i++ ) {
        printf ( "%s %d\n", dev_list.device[i].deviceId, dev_list.device[i].deviceType );
    }
    // use first device
    lpms = manager->addSensor ( DEVICE_LPMS_U, dev_list.getDeviceId ( 0 ) );

    /////////////////////////////////////////////////////////////////
    //
    std::string pipe ( __pipe_prefix + pipe_name );
    xddp_sock = open ( pipe.c_str(), O_WRONLY );

    if ( xddp_sock < 0 ) {
        printf ( "%s : %s\n", pipe.c_str(), strerror ( errno ) );
    } else {
        std::cout << "Using "<< pipe << std::endl;
    }

    /////////////////////////////////////////////////////////////////
    //
    imuPub = new ImuPub ( std::string("tcp://*:10101"), std::string("Imu") );

    if ( use_cb ) {
        lpms->setCallback ( cb_hook );
    } else {
        loop = true;
    }

}

void ImuHandler::operator() () const {
    ImuData imu_data;
    char    id[64];

    lpms->getDeviceId ( id );

    while ( loop ) {

        // Checks, if conncted
        if ( lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
                lpms->hasImuData() ) {

            // Reads quaternion data
            imu_data = lpms->getCurrentData();
            lpms_cb ( imu_data, id );

        }
        SLEEP ( milliseconds ( 1 ) );
    }
}

} // namespace

////////////////////////////////////////////////////
// Main
// udevadm info -a -p $(udevadm info -q path -n ttyUSB0) | egrep -i "ATTRS{serial}|ATTRS{idVendor}|ATTRS{idProduct}" -m 3
////////////////////////////////////////////////////

using std::placeholders::_1;
using std::placeholders::_2;

int main ( int argc, char * const argv[] ) {
    bool use_cb = true;
    ImuHandler imu;
    main_common ( &argc, &argv, shutdown );
    lpms_cb = std::bind ( &ImuHandler::imu_data_cb, &imu, _1, _2 );
    imu.setup ( std::string ( "Lpms_imu" ),use_cb );

    //lpms_cb(ImuData(), "test_imu_cb");

    if ( ! use_cb ) {
        std::thread t ( std::ref ( imu ) );
        while ( main_loop ) {
            SLEEP ( seconds ( 1 ) );
        }
        imu.stop();
        t.join();
    } else {
        while ( main_loop ) {
            SLEEP ( seconds ( 1 ) );
        }
        imu.stop();
    }
    return 0;
}



// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
