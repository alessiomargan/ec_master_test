#include <stdio.h>
#include <stdint.h>

#include <iit/ecat/advr/zmq_publisher.h>

#include <iit/ecat/advr/esc.h>
#include <iit/ecat/advr/ft6_esc.h>
#include <iit/ecat/advr/test_esc.h>

typedef Publisher<iit::ecat::advr::TestEscPdoTypes::pdo_rx> TestPub;
typedef Publisher<iit::ecat::advr::Ft6EscPdoTypes::pdo_rx> FtPub;
typedef Publisher<iit::ecat::advr::McEscPdoTypes::pdo_rx> McPub;

extern int run_loop;
extern int main_common(void);



int main(int argc, char **argv)
{
    PubMap_t        zmap;
    Abs_Publisher * zpub;
    
    main_common();
    
    int base_port = 9000;
    std::string uri("tcp://*:");
    
    base_port++;
    zpub = new TestPub(uri+std::to_string(base_port)); 
    if ( zpub->open_pipe("ESC_test_pos1") == 0 ) { zmap[base_port] = zpub; }
    else { delete zpub; }
    
    base_port++;
    zpub = new FtPub(uri+std::to_string(base_port));
    if ( zpub->open_pipe("Ft6ESC_0") == 0 ) { zmap[base_port] = zpub; }
    else { delete zpub; }
    
    base_port++;
    zpub = new McPub(uri+std::to_string(base_port));
    if ( zpub->open_pipe("HpESC_1000") == 0 ) { zmap[base_port] = zpub; }
    else { delete zpub; }
    
    
    while (run_loop) {
        for (auto it = zmap.begin(); it != zmap.end(); it++) { it->second->publish(); }
    }
    
    for (auto it = zmap.begin(); it != zmap.end(); it++) { delete it->second; }
    
    return 1;
}
