#include <cstdio>
#include "ati_iface.h"
#include <thread>
#include <chrono>
#include "giel.h"

int main()
{
    Ati_Sens ati;
    ati.start_thread();
    sleep(1);

    while(1)
    {
        ati_log_t last{};
        ati.get_last_sample(last);
        char buf[128];
        last.sprint(buf, 128);
        std::cout << buf ;
        MILLISLEEP(25);
    }
}
