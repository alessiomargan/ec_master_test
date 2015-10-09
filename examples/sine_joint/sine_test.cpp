#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <exception>

#include <ec_boards_sine.h>


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
    
    threads["boards_ctrl"] = new Ec_Boards_sine(argv[1]);
    threads["boards_ctrl"]->create(false);

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


