#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <exception>

#include <ec_boards_coman_test.h>
//#include <iit/advr/zmq_publisher.h>

extern void main_common(void);
extern int looping(void);

////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main(int argc, char *argv[]) try {

    std::map<std::string, Thread_hook*> threads;

    if ( argc != 2) {
	printf("Usage: %s config.yaml\n", argv[0]);
        return 0;
    }

    main_common();
    
    threads["EC_boards_coman_test"] = new EC_boards_coman_test(argv[1]);
    threads["EC_boards_coman_test"]->create(true,2);

#if 0
    // ZMQ_pub wait for pipe creation
    while ( ! dynamic_cast<Ec_Thread_Boards_base*>(threads["EC_boards_joint_joy"])->init_OK() ) {
        sleep(1);
    }
    threads["ZMQ_pub"] = new iit::ZMQ_Pub_thread();
    threads["ZMQ_pub"]->create(false,3);
#endif

    while (looping()) {
        sleep(1);
    }

    for (auto it = threads.begin(); it != threads.end(); it++) {
        it->second->stop();
        it->second->join();
        delete it->second;
    }

    std::cout << "Exit main" << std::endl;

    return 0;

} catch (std::exception& e) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}


