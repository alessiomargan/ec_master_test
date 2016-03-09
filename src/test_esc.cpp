#include <iit/ecat/advr/test_esc.h>
#include <string>

using namespace iit::ecat;
using namespace iit::ecat::advr;

//template<class EscPDOTypes, class EscSDOTypes>
//typename BasicEscWrapper<EscPDOTypes, EscSDOTypes>::sdo_t    BasicEscWrapper<EscPDOTypes, EscSDOTypes>::sdo;

static const objd_t source_SDOs[] = {
    // SD0 0x8000
    { 0x8000, 0x1, DTYPE_INTEGER32,     32, ATYPE_RW,    "par_1"           ,0  },
    { 0x8000, 0x2, DTYPE_INTEGER32,     32, ATYPE_RW,    "par_2"           ,0  },

    {0, 0, 0, 0, 0, 0}

};


void TestESC::init_SDOs ( void ) {

    int objd_num, i = 0;

    objd_num = sizeof ( source_SDOs ) /sizeof ( objd_t );
    SDOs = new objd_t [objd_num];

    memcpy ( ( void* ) SDOs, source_SDOs, sizeof ( source_SDOs ) );

    SDOs[i++].data = ( void* ) &TestESC::sdo.par_1;
    SDOs[i++].data = ( void* ) &TestESC::sdo.par_2;

    SDOs[i++].data = 0;

    assert ( objd_num == i );
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
