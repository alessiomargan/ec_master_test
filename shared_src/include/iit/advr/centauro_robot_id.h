#ifndef __CENTAURO_ROBOT_ID_H__
#define __CENTAURO_ROBOT_ID_H__

#include <initializer_list>
#include <algorithm>
#include <map>

namespace iit {
namespace ecat {
namespace advr {
namespace centauro {


enum Robot_IDs : int {
    // waist
    WAIST_Y	= 1,

    // right arm
    RA_SH_1	= 2,
    RA_SH_2	= 3,
    RA_SH_3	= 4,
    RA_EL	= 5,
    RA_WR_1	= 6,
    RA_WR_2	= 7,
    RA_WR_3	= 8,
    RA_FT	= 80,
    RA_HA	= 99,

    // left arm
    LA_SH_1	=  9,
    LA_SH_2	= 10,
    LA_SH_3	= 11,
    LA_EL	= 12,
    LA_WR_1	= 13,
    LA_WR_2	= 14,
    LA_WR_3	= 15,
    LA_FT	= 81,
    LA_HA	= 91,

};

const std::vector<int> robot_waist_ids = std::initializer_list<int> {
    // waist
    WAIST_Y,
};


const std::vector<int> robot_right_arm_ids = std::initializer_list<int> {
    // right arm
    RA_SH_1,
    RA_SH_2,
    RA_SH_3,
    RA_EL,
    RA_WR_1,
    RA_WR_2,
    RA_WR_3,
    RA_HA,
};

const std::vector<int> robot_left_arm_ids = std::initializer_list<int> {
    // left arm
    LA_SH_1,
    LA_SH_2,
    LA_SH_3,
    LA_EL,
    LA_WR_1,
    LA_WR_2,
    LA_WR_3,
    LA_HA,
};

const std::vector<int> robot_fts_ids = std::initializer_list<int> {
    RA_FT, LA_FT
};

const std::vector<int> robot_mcs_ids = std::initializer_list<int> {
    // waist
    WAIST_Y,// right arm
    RA_SH_1, RA_SH_2, RA_SH_3, RA_EL, RA_WR_1, RA_WR_2, RA_WR_3, RA_HA,
    // left arm
    LA_SH_1, LA_SH_2, LA_SH_3, LA_EL, LA_WR_1, LA_WR_2, LA_WR_3, LA_HA,

};

std::map<int, float> robot_ids_home_pos_deg = {

    {WAIST_Y,   0.0},

    {RA_SH_1,  0.0}, {RA_SH_2, 30.0}, {RA_SH_3, 30.0}, {RA_EL, 45.0},
    {RA_WR_1,  0.0}, {RA_WR_2, 45.0},  {RA_WR_3,  0.0}, {RA_HA, 0.0},

    {LA_SH_1,  0.0}, {LA_SH_2,-30.0}, {LA_SH_3,-30.0}, {LA_EL,-45.0},
    {LA_WR_1,  0.0}, {LA_WR_2, -45.0}, {LA_WR_3,  0.0}, {LA_HA,  0.0}

};

std::map<int, float> robot_ids_test_pos_deg = {

    {WAIST_Y, 0.0},
    
    {RA_SH_1, 90.0}, {RA_SH_2, 100.0}, {RA_SH_3, 60.0}, {RA_EL, 70.0},
    {RA_WR_1,-60.0}, {RA_WR_2,  45.0}, {RA_WR_3, -60.0}, {RA_HA,   0.0},
//    {RA_WR_1,  0.0}, {RA_WR_2,   0.0}, {RA_WR_3,  0.0}, {RA_HA,  0.0},

    {LA_SH_1,-90.0}, {LA_SH_2,-100.0}, {LA_SH_3,-60.0}, {LA_EL,-70.0},
    {LA_WR_1, 60.0}, {LA_WR_2,  -45.0}, {LA_WR_3, 60.0}, {LA_HA,   0.0}
//    {LA_WR_1,  0.0}, {LA_WR_2,   0.0}, {LA_WR_3,  0.0}, {LA_HA,  0.0}

};

std::map<int, float> robot_ids_zero_pos_deg = {

    {WAIST_Y,   0.0},

    {RA_SH_1,  0.0}, {RA_SH_2, 10.0}, {RA_SH_3,20.0}, {RA_EL, 0.0},
    {RA_WR_1,  0.0}, {RA_WR_2,  0.0}, {RA_WR_3, 0.0}, {RA_HA, 0.0},

    {LA_SH_1,  0.0}, {LA_SH_2,-10.0}, {LA_SH_3,-20.0}, {LA_EL, 0.0},
    {LA_WR_1,  0.0}, {LA_WR_2,  0.0}, {LA_WR_3,  0.0}, {LA_HA, 0.0}

};

std::map<int, float> robot_ids_up_pos_deg = {

    {WAIST_Y, 45.0},

    {RA_SH_1, 45.0}, {RA_SH_2, 10.0}, {RA_SH_3,50.0}, {RA_EL, 90.0},
    {RA_WR_1,  0.0}, {RA_WR_2, 60.0}, {RA_WR_3, 0.0}, {RA_HA,  0.0},

    {LA_SH_1,-45.0}, {LA_SH_2,-10.0}, {LA_SH_3,-50.0}, {LA_EL,-90.0},
    {LA_WR_1,  0.0}, {LA_WR_2,-60.0}, {LA_WR_3,  0.0}, {LA_HA,  0.0}

};

std::map<int, float> robot_ids_extend_pos_deg = {

    {WAIST_Y,  -45.0},

    {RA_SH_1,  0.0}, {RA_SH_2, 90.0}, {RA_SH_3, 0.0}, {RA_EL, 0.0},
    {RA_WR_1,  0.0}, {RA_WR_2,  0.0}, {RA_WR_3, 0.0}, {RA_HA, 0.0},

    {LA_SH_1,  0.0}, {LA_SH_2,-90.0}, {LA_SH_3, 0.0}, {LA_EL, 0.0},
    {LA_WR_1,  0.0}, {LA_WR_2,  0.0}, {LA_WR_3, 0.0}, {LA_HA, 0.0}

};


}
}
}
}

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
