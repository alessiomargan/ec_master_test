#include <iit/ecat/advr/foot_sensor_esc.h>

using namespace iit::ecat;
using namespace iit::ecat::advr;

static const char acName6000[] = "Inputs";
static const char acName6000_0[] = "Number of Elements";
static const char acName6000_fault[] = "fault";
static const char acName6000_rtt[] = "rtt";

static const char acName6000_force1A[] = "Force1A";
static const char acName6000_force1B[] = "Force1B";
static const char acName6000_force1C[] = "Force1C";
static const char acName6000_force1D[] = "Force1D";
static const char acName6000_force1E[] = "Force1E";
static const char acName6000_force1F[] = "Force1F";
static const char acName6000_force1G[] = "Force1G";
static const char acName6000_force1H[] = "Force1H";
static const char acName6000_force2A[] = "Force2A";
static const char acName6000_force2B[] = "Force2B";
static const char acName6000_force2C[] = "Force2C";
static const char acName6000_force2D[] = "Force2D";
static const char acName6000_force2E[] = "Force2E";
static const char acName6000_force2F[] = "Force2F";
static const char acName6000_force2G[] = "Force2G";
static const char acName6000_force2H[] = "Force2H";
static const char acName6000_force3A[] = "Force3A";
static const char acName6000_force3B[] = "Force3B";
static const char acName6000_force3C[] = "Force3C";
static const char acName6000_force3D[] = "Force3D";
static const char acName6000_force3E[] = "Force3E";
static const char acName6000_force3F[] = "Force3F";
static const char acName6000_force3G[] = "Force3G";
static const char acName6000_force3H[] = "Force3H";
static const char acName6000_force4A[] = "Force4A";
static const char acName6000_force4B[] = "Force4B";
static const char acName6000_force4C[] = "Force4C";
static const char acName6000_force4D[] = "Force4D";
static const char acName6000_force4E[] = "Force4E";
static const char acName6000_force4F[] = "Force4F";
static const char acName6000_force4G[] = "Force4G";
static const char acName6000_force4H[] = "Force4H";
static const char acName6000_force5A[] = "Force5A";
static const char acName6000_force5B[] = "Force5B";
static const char acName6000_force5C[] = "Force5C";
static const char acName6000_force5D[] = "Force5D";
static const char acName6000_force5E[] = "Force5E";
static const char acName6000_force5F[] = "Force5F";
static const char acName6000_force5G[] = "Force5G";
static const char acName6000_force5H[] = "Force5H";
static const char acName6000_force6A[] = "Force6A";
static const char acName6000_force6B[] = "Force6B";
static const char acName6000_force6C[] = "Force6C";
static const char acName6000_force6D[] = "Force6D";
static const char acName6000_force6E[] = "Force6E";
static const char acName6000_force6F[] = "Force6F";
static const char acName6000_force6G[] = "Force6G";
static const char acName6000_force6H[] = "Force6H";
static const char acName6000_force7A[] = "Force7A";
static const char acName6000_force7B[] = "Force7B";
static const char acName6000_force7C[] = "Force7C";
static const char acName6000_force7D[] = "Force7D";
static const char acName6000_force7E[] = "Force7E";
static const char acName6000_force7F[] = "Force7F";
static const char acName6000_force7G[] = "Force7G";
static const char acName6000_force7H[] = "Force7H";
static const char acName6000_force8A[] = "Force8A";
static const char acName6000_force8B[] = "Force8B";
static const char acName6000_force8C[] = "Force8C";
static const char acName6000_force8D[] = "Force8D";
static const char acName6000_force8E[] = "Force8E";
static const char acName6000_force8F[] = "Force8F";
static const char acName6000_force8G[] = "Force8G";
static const char acName6000_force8H[] = "Force8H";
static const char acName6000_force9A[] = "Force9A";
static const char acName6000_force9B[] = "Force9B";
static const char acName6000_force9C[] = "Force9C";
static const char acName6000_force9D[] = "Force9D";
static const char acName6000_force9E[] = "Force9E";
static const char acName6000_force9F[] = "Force9F";
static const char acName6000_force9G[] = "Force9G";
static const char acName6000_force9H[] = "Force9H";
static const char acName6000_force10A[] = "Force10A";
static const char acName6000_force10B[] = "Force10B";
static const char acName6000_force10C[] = "Force10C";
static const char acName6000_force10D[] = "Force10D";
static const char acName6000_force10E[] = "Force10E";
static const char acName6000_force10F[] = "Force10F";
static const char acName6000_force10G[] = "Force10G";
static const char acName6000_force10H[] = "Force10H";
static const char acName6000_force11A[] = "Force11A";
static const char acName6000_force11B[] = "Force11B";
static const char acName6000_force11C[] = "Force11C";
static const char acName6000_force11D[] = "Force11D";
static const char acName6000_force11E[] = "Force11E";
static const char acName6000_force11F[] = "Force11F";
static const char acName6000_force11G[] = "Force11G";
static const char acName6000_force11H[] = "Force11H";
static const char acName6000_force12A[] = "Force12A";
static const char acName6000_force12B[] = "Force12B";
static const char acName6000_force12C[] = "Force12C";
static const char acName6000_force12D[] = "Force12D";
static const char acName6000_force12E[] = "Force12E";
static const char acName6000_force12F[] = "Force1F";
static const char acName6000_force12G[] = "Force12G";
static const char acName6000_force12H[] = "Force12H";
static const char acName6000_force13A[] = "Force13A";
static const char acName6000_force13B[] = "Force13B";
static const char acName6000_force13C[] = "Force13C";
static const char acName6000_force13D[] = "Force13D";
static const char acName6000_force13E[] = "Force13E";
static const char acName6000_force13F[] = "Force13F";
static const char acName6000_force13G[] = "Force13G";
static const char acName6000_force13H[] = "Force13H";
static const char acName6000_force14A[] = "Force14A";
static const char acName6000_force14B[] = "Force14B";
static const char acName6000_force14C[] = "Force14C";
static const char acName6000_force14D[] = "Force14D";
static const char acName6000_force14E[] = "Force14E";
static const char acName6000_force14F[] = "Force14F";
static const char acName6000_force14G[] = "Force14G";
static const char acName6000_force14H[] = "Force14H";
static const char acName6000_force15A[] = "Force15A";
static const char acName6000_force15B[] = "Force15B";
static const char acName6000_force15C[] = "Force15C";
static const char acName6000_force15D[] = "Force15D";
static const char acName6000_force15E[] = "Force15E";
static const char acName6000_force15F[] = "Force15F";
static const char acName6000_force15G[] = "Force15G";
static const char acName6000_force15H[] = "Force15H";
static const char acName6000_force16A[] = "Force16A";
static const char acName6000_force16B[] = "Force16B";
static const char acName6000_force16C[] = "Force16C";
static const char acName6000_force16D[] = "Force16D";
static const char acName6000_force16E[] = "Force16E";
static const char acName6000_force16F[] = "Force16F";
static const char acName6000_force16G[] = "Force16G";
static const char acName6000_force16H[] = "Force16H";

static const char acName7000[] = "Outputs";
static const char acName7000_0[] = "Number of Elements";
static const char acName7000_1[] = "pos_ref";
static const char acName7000_2[] = "tor_ref";
static const char acName7000_3[] = "direct_ref";
static const char acName7000_4[] = "ts";

static const char acName8000[] = "Flash Parameter";
static const char acName8000_0[] = "Number of Elements";
static const char acName8000_1[] = "Block control";
static const char acName8000_2[] = "Num Av Samples";
static const char acName8000_3[] = "Cal offset 0";
static const char acName8000_4[] = "Cal offset 1";
static const char acName8000_5[] = "Cal offset 2";
static const char acName8000_6[] = "Cal offset 3";
static const char acName8000_7[] = "Cal offset 4";
static const char acName8000_8[] = "Cal offset 5";
static const char acName8000_9[] = "matrix_r1_c1";
static const char acName8000_10[] = "matrix_r1_c2";
static const char acName8000_11[] = "matrix_r1_c3";
static const char acName8000_12[] = "matrix_r1_c4";
static const char acName8000_13[] = "matrix_r1_c5";
static const char acName8000_14[] = "matrix_r1_c6";
static const char acName8000_15[] = "Sensor_number";
static const char acName8000_16[] = "Sensor_robot_id";

static const char acName8001[] = "Parameter";
static const char acName8001_1[] = "fw_ver";
static const char acName8001_2[] = "ack_board_faults";

static const char acName8001_3[] = "Matrix c1";
static const char acName8001_4[] = "Matrix c2";
static const char acName8001_5[] = "Matrix c3";
static const char acName8001_6[] = "Matrix c4";
static const char acName8001_7[] = "Matrix c5";
static const char acName8001_8[] = "Matrix c6";

static const char acName8001_9[] = "flash_params_cmd";
static const char acName8001_10[] = "flash_params_cmd_ack";




static const iit::ecat::objd_t source_SDOs[] = {

    // SDO6000[] =
    {0x6000, 0x1, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force1A, 0},
    {0x6000, 0x2, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force1B, 0},
    {0x6000, 0x3, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force1C, 0},
    {0x6000, 0x4, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force1D, 0},
    {0x6000, 0x5, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force1E, 0},
    {0x6000, 0x6, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force1F, 0},
    {0x6000, 0x7, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force1G, 0},
    {0x6000, 0x8, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force1H, 0},
    {0x6000, 0x9, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force2A, 0},
    {0x6000, 0xa, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force2B, 0},
    {0x6000, 0xb, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force2C, 0},
    {0x6000, 0xc, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force2D, 0},
    {0x6000, 0xd, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force2E, 0},
    {0x6000, 0xe, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force2F, 0},
    {0x6000, 0xf, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force2G, 0},
    {0x6000, 0x10, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force2H, 0},
    {0x6000, 0x11, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force3A, 0},
    {0x6000, 0x12, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force3B, 0},
    {0x6000, 0x13, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force3C, 0},
    {0x6000, 0x14, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force3D, 0},
    {0x6000, 0x15, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force3E, 0},
    {0x6000, 0x16, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force3F, 0},
    {0x6000, 0x17, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force3G, 0},
    {0x6000, 0x18, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force3H, 0},
    {0x6000, 0x19, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force4A, 0},
    {0x6000, 0x1a, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force4B, 0},
    {0x6000, 0x1b, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force4C, 0},
    {0x6000, 0x1c, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force4D, 0},
    {0x6000, 0x1d, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force4E, 0},
    {0x6000, 0x1e, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force4F, 0},
    {0x6000, 0x1f, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force4G, 0},
    {0x6000, 0x20, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force4H, 0},
    {0x6000, 0x21, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force5A, 0},
    {0x6000, 0x22, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force5B, 0},
    {0x6000, 0x23, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force5C, 0},
    {0x6000, 0x24, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force5D, 0},
    {0x6000, 0x25, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force5E, 0},
    {0x6000, 0x26, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force5F, 0},
    {0x6000, 0x27, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force5G, 0},
    {0x6000, 0x28, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force5H, 0},
    {0x6000, 0x29, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force6A, 0},
    {0x6000, 0x2a, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force6B, 0},
    {0x6000, 0x2b, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force6C, 0},
    {0x6000, 0x2c, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force6D, 0},
    {0x6000, 0x2d, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force6E, 0},
    {0x6000, 0x2e, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force6F, 0},
    {0x6000, 0x2f, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force6G, 0},
    {0x6000, 0x30, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force6H, 0},
    {0x6000, 0x31, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force7A, 0},
    {0x6000, 0x32, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force7B, 0},
    {0x6000, 0x33, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force7C, 0},
    {0x6000, 0x34, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force7D, 0},
    {0x6000, 0x35, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force7E, 0},
    {0x6000, 0x36, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force7F, 0},
    {0x6000, 0x37, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force7G, 0},
    {0x6000, 0x38, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force7H, 0},
    {0x6000, 0x39, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force8A, 0},
    {0x6000, 0x3a, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force8B, 0},
    {0x6000, 0x3b, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force8C, 0},
    {0x6000, 0x3c, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force8D, 0},
    {0x6000, 0x3d, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force8E, 0},
    {0x6000, 0x3e, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force8F, 0},
    {0x6000, 0x3f, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force8G, 0},
    {0x6000, 0x40, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force8H, 0},
    {0x6000, 0x41, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force9A, 0},
    {0x6000, 0x42, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force9B, 0},
    {0x6000, 0x43, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force9C, 0},
    {0x6000, 0x44, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force9D, 0},
    {0x6000, 0x45, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force9E, 0},
    {0x6000, 0x46, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force9F, 0},
    {0x6000, 0x47, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force9G, 0},
    {0x6000, 0x48, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force9H, 0},
    {0x6000, 0x49, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force10A, 0},
    {0x6000, 0x4a, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force10B, 0},
    {0x6000, 0x4b, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force10C, 0},
    {0x6000, 0x4c, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force10D, 0},
    {0x6000, 0x4d, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force10E, 0},
    {0x6000, 0x4e, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force10F, 0},
    {0x6000, 0x4f, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force10G, 0},
    {0x6000, 0x50, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force10H, 0},
    {0x6000, 0x51, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force11A, 0},
    {0x6000, 0x52, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force11B, 0},
    {0x6000, 0x53, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force11C, 0},
    {0x6000, 0x54, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force11D, 0},
    {0x6000, 0x55, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force11E, 0},
    {0x6000, 0x56, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force11F, 0},
    {0x6000, 0x57, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force11G, 0},
    {0x6000, 0x58, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force11H, 0},
    {0x6000, 0x59, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force12A, 0},
    {0x6000, 0x5a, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force12B, 0},
    {0x6000, 0x5b, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force12C, 0},
    {0x6000, 0x5c, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force12D, 0},
    {0x6000, 0x5d, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force12E, 0},
    {0x6000, 0x5e, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force12F, 0},
    {0x6000, 0x5f, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force12G, 0},
    {0x6000, 0x60, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force12H, 0},
    {0x6000, 0x61, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force13A, 0},
    {0x6000, 0x62, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force13B, 0},
    {0x6000, 0x63, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force13C, 0},
    {0x6000, 0x64, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force13D, 0},
    {0x6000, 0x65, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force13E, 0},
    {0x6000, 0x66, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force13F, 0},
    {0x6000, 0x67, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force13G, 0},
    {0x6000, 0x68, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force13H, 0},
    {0x6000, 0x69, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force14A, 0},
    {0x6000, 0x6a, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force14B, 0},
    {0x6000, 0x6b, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force14C, 0},
    {0x6000, 0x6c, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force14D, 0},
    {0x6000, 0x6d, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force14E, 0},
    {0x6000, 0x6e, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force14F, 0},
    {0x6000, 0x6f, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force14G, 0},
    {0x6000, 0x70, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force14H, 0},
    {0x6000, 0x71, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force15A, 0},
    {0x6000, 0x72, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force15B, 0},
    {0x6000, 0x73, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force15C, 0},
    {0x6000, 0x74, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force15D, 0},
    {0x6000, 0x75, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force15E, 0},
    {0x6000, 0x76, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force15F, 0},
    {0x6000, 0x77, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force15G, 0},
    {0x6000, 0x78, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force15H, 0},
    {0x6000, 0x79, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force16A, 0},
    {0x6000, 0x7a, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force16B, 0},
    {0x6000, 0x7b, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force16C, 0},
    {0x6000, 0x7c, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force16D, 0},
    {0x6000, 0x7d, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force16E, 0},
    {0x6000, 0x7e, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force16F, 0},
    {0x6000, 0x7f, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force16G, 0},
    {0x6000, 0x80, DTYPE_UNSIGNED8,   8, ATYPE_RO, acName6000_force16H, 0},
    {0x6000, 0x81, DTYPE_UNSIGNED16,  16, ATYPE_RO, acName6000_fault, 0},
    {0x6000, 0x82, DTYPE_UNSIGNED16,  16, ATYPE_RO, acName6000_rtt, 0},
    // SDO7000[] =
    {0x7000, 0x1, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7000_4, 0},
    // SDO8000[] =
    {0x8000, 0x1, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_1,  0},
    {0x8000, 0x2, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_2,  0},
    {0x8000, 0x3, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_3,  0},
    {0x8000, 0x4, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_4,  0},
    {0x8000, 0x5, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_5,  0},
    {0x8000, 0x6, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_6,  0},
    {0x8000, 0x7, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_7,  0},
    {0x8000, 0x8, DTYPE_INTEGER32,     32, ATYPE_RW, acName8000_8,  0},
    {0x8000, 0x9, DTYPE_REAL32,        32, ATYPE_RW, acName8000_9,  0},
    {0x8000, 0xa, DTYPE_REAL32,        32, ATYPE_RW, acName8000_10, 0},
    {0x8000, 0xb, DTYPE_REAL32,        32, ATYPE_RW, acName8000_11, 0},
    {0x8000, 0xc, DTYPE_REAL32,        32, ATYPE_RW, acName8000_12, 0},
    {0x8000, 0xd, DTYPE_REAL32,        32, ATYPE_RW, acName8000_13, 0},
    {0x8000, 0xe, DTYPE_REAL32,        32, ATYPE_RW, acName8000_14, 0},
    {0x8000, 0xf,  DTYPE_INTEGER16,    16, ATYPE_RW, acName8000_15, 0},
    {0x8000, 0x10, DTYPE_INTEGER16,    16, ATYPE_RW, acName8000_16, 0},
    // SDO8001[] =
    {0x8001, 0x1, DTYPE_VISIBLE_STRING,   64, ATYPE_RO, acName8001_1, 0},
    {0x8001, 0x2, DTYPE_INTEGER16,        16, ATYPE_RW, acName8001_2, 0},
    {0x8001, 0x3, DTYPE_REAL32,           32, ATYPE_RW, acName8001_3, 0},
    {0x8001, 0x4, DTYPE_REAL32,           32, ATYPE_RW, acName8001_4, 0},
    {0x8001, 0x5, DTYPE_REAL32,           32, ATYPE_RW, acName8001_5, 0},
    {0x8001, 0x6, DTYPE_REAL32,           32, ATYPE_RW, acName8001_6, 0},
    {0x8001, 0x7, DTYPE_REAL32,           32, ATYPE_RW, acName8001_7, 0},
    {0x8001, 0x8, DTYPE_REAL32,           32, ATYPE_RW, acName8001_8, 0},
    {0x8001, 0x9, DTYPE_UNSIGNED16,        16, ATYPE_RW, acName8001_9, 0},
    {0x8001, 0xa, DTYPE_UNSIGNED16,        16, ATYPE_RO, acName8001_10, 0},

    {0, 0, 0, 0, 0, 0, 0 }
};



void FootSensorESC::init_SDOs ( void ) {

    int objd_num, i = 0;

    objd_num = sizeof ( source_SDOs ) /sizeof ( objd_t );
    SDOs = new objd_t [objd_num];

    memcpy ( ( void* ) SDOs, source_SDOs, sizeof ( source_SDOs ) );

    // 0x6000
    for(int j = 0; j < SENSOR_NUMBER; j++) {
        SDOs[i++].data = ( void* ) &FootSensorESC::rx_pdo.forceXY[j];
    }
    SDOs[i++].data = ( void* ) &FootSensorESC::rx_pdo.fault;
    SDOs[i++].data = ( void* ) &FootSensorESC::rx_pdo.rtt;
    // 0x7000
    SDOs[i++].data = ( void* ) &FootSensorESC::tx_pdo.ts;
    // 0x8000
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.Block_control;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.NumAvSamples;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.calibration_offset0;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.calibration_offset1;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.calibration_offset2;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.calibration_offset3;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.calibration_offset4;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.calibration_offset5;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.matrix_r1_c1;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.matrix_r1_c2;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.matrix_r1_c3;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.matrix_r1_c4;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.matrix_r1_c5;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.matrix_r1_c6;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.sensor_number;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.sensor_robot_id;
    // 0x8001
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.firmware_version;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.ack_board_fault;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.matrix_rn_c1;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.matrix_rn_c2;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.matrix_rn_c3;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.matrix_rn_c4;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.matrix_rn_c5;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.matrix_rn_c6;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.flash_params_cmd;
    SDOs[i++].data = ( void* ) &FootSensorESC::sdo.flash_params_cmd_ack;

    SDOs[i++].data = 0;

    assert ( objd_num == i );
}
