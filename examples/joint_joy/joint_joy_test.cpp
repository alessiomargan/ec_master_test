#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <exception>

#include <ec_boards_joint_joy.h>
#include <iit/advr/zmq_publisher.h>

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
    
    threads["EC_boards_joint_joy"] = new EC_boards_joint_joy(argv[1]);
    threads["EC_boards_joint_joy"]->create(true,2);

    // ZMQ_pub wait for pipe creation
    while ( ! dynamic_cast<Ec_Thread_Boards_base*>(threads["EC_boards_joint_joy"])->init_OK() ) {
        sleep(1);
    }
    threads["ZMQ_pub"] = new iit::ecat::advr::ZMQ_Pub_thread();
    threads["ZMQ_pub"]->create(false,3);

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


