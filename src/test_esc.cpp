#include <iit/ecat/advr/test_esc.h>
#include <string>

using namespace iit::ecat::advr;

template<class ESCParamTypes>
typename TestESCtemp<ESCParamTypes>::flash_param_t    TestESCtemp<ESCParamTypes>::flash_param;

const objd_t TestESC::SDOs[] =
{
    // SD0 0x8000
    { 0x8000, 0x1, DTYPE_INTEGER32,     32, ATYPE_RW,    "par_1"           ,(void*)&TestESC::flash_param.par_1  },
    { 0x8000, 0x2, DTYPE_INTEGER32,     32, ATYPE_RW,    "par_2"           ,(void*)&TestESC::flash_param.par_2  },
    
    {0, 0, 0, 0, 0, 0}

};



