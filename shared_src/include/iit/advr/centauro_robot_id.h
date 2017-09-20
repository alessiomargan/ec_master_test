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

    // right front leg
    RFL_H_Y = 41,
    RFL_H_P = 42,
    RFL_K   = 43,
    RFL_A_P = 44,
    RFL_A_Y = 45,
    RFL_W   = 46,

    // right hind leg
    RHL_H_Y = 51,
    RHL_H_P = 52,
    RHL_K   = 53,
    RHL_A_P = 54,
    RHL_A_Y = 55,
    RHL_W   = 56,

    // left hind leg
    LHL_H_Y = 61,
    LHL_H_P = 62,
    LHL_K   = 63,
    LHL_A_P = 64,
    LHL_A_Y = 65,
    LHL_W   = 66,

    // left front leg
    LFL_H_Y = 71,
    LFL_H_P = 72,
    LFL_K   = 73,
    LFL_A_P = 74,
    LFL_A_Y = 75,
    LFL_W   = 76,


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

const std::vector<int> robot_left_front_leg_ids = std::initializer_list<int> {
    // left front leg
    LFL_H_Y,
    LFL_H_P,
    LFL_K  ,
    LFL_A_P,
    LFL_A_Y,
    LFL_W  ,
};

const std::vector<int> robot_left_hind_leg_ids = std::initializer_list<int> {
    // left front leg
    LHL_H_Y,
    LHL_H_P,
    LHL_K  ,
    LHL_A_P,
    LHL_A_Y,
    LHL_W  ,
};

const std::vector<int> robot_right_front_leg_ids = std::initializer_list<int> {
    // left front leg
    RFL_H_Y,
    RFL_H_P,
    RFL_K  ,
    RFL_A_P,
    RFL_A_Y,
    RFL_W  ,
};

const std::vector<int> robot_right_hind_leg_ids = std::initializer_list<int> {
    // left front leg
    RHL_H_Y,
    RHL_H_P,
    RHL_K  ,
    RHL_A_P,
    RHL_A_Y,
    RHL_W  ,
};

const std::vector<int> robot_fts_ids = std::initializer_list<int> {
    RA_FT, LA_FT
};

const std::vector<int> robot_mcs_ids = std::initializer_list<int> {
    // waist
    WAIST_Y,
    // right arm
    RA_SH_1, RA_SH_2, RA_SH_3, RA_EL, RA_WR_1, RA_WR_2, RA_WR_3, RA_HA,
    // left arm
    LA_SH_1, LA_SH_2, LA_SH_3, LA_EL, LA_WR_1, LA_WR_2, LA_WR_3, LA_HA,
    // left front leg
    LFL_H_Y, LFL_H_P, LFL_K, LFL_A_P, LFL_A_Y, LFL_W,
    // left hind leg
    LHL_H_Y, LHL_H_P, LHL_K, LHL_A_P, LHL_A_Y, LHL_W,
    // right front leg
    RFL_H_Y, RFL_H_P, RFL_K, RFL_A_P, RFL_A_Y, RFL_W,
    // right hind leg
    RHL_H_Y, RHL_H_P, RHL_K, RHL_A_P, RHL_A_Y, RHL_W,

};

const std::map<int, float> robot_ids_home_pos_deg = {

    {WAIST_Y,   0.0},

    {RA_SH_1,  0.0}, {RA_SH_2, 30.0}, {RA_SH_3, 30.0}, {RA_EL, 45.0},
    {RA_WR_1,  0.0}, {RA_WR_2, 45.0}, {RA_WR_3,  0.0}, {RA_HA, 0.0},

    {LA_SH_1,  0.0}, {LA_SH_2,-30.0}, {LA_SH_3,-30.0}, {LA_EL,-45.0},
    {LA_WR_1,  0.0}, {LA_WR_2,-45.0}, {LA_WR_3,  0.0}, {LA_HA,  0.0},
    
    {LFL_H_Y, 45.0}, {LFL_H_P,  45.0}, {LFL_K,   0.0}, {LFL_A_P, 0.0}, {LFL_A_Y, 0.0}, {LFL_W, 0.0},
    {LHL_H_Y, 45.0}, {LHL_H_P,  45.0}, {LHL_K,   0.0}, {LHL_A_P, 0.0}, {LHL_A_Y, 0.0}, {LHL_W, 0.0},
    {RFL_H_Y, 45.0}, {RFL_H_P,  45.0}, {RFL_K,   0.0}, {RFL_A_P, 0.0}, {RFL_A_Y, 0.0}, {RFL_W, 0.0},
    {RHL_H_Y, 45.0}, {RHL_H_P,  45.0}, {RHL_K,   0.0}, {RHL_A_P, 0.0}, {RHL_A_Y, 0.0}, {RHL_W, 0.0},

};

const std::map<int, float> robot_ids_test_pos_deg = {

    {WAIST_Y, 0.0},
    
    {RA_SH_1, 90.0}, {RA_SH_2, 100.0}, {RA_SH_3, 60.0}, {RA_EL, 70.0},
    {RA_WR_1,-60.0}, {RA_WR_2,  45.0}, {RA_WR_3, -60.0}, {RA_HA,   0.0},
//    {RA_WR_1,  0.0}, {RA_WR_2,   0.0}, {RA_WR_3,  0.0}, {RA_HA,  0.0},

    {LA_SH_1,-90.0}, {LA_SH_2,-100.0}, {LA_SH_3,-60.0}, {LA_EL,-70.0},
    {LA_WR_1, 60.0}, {LA_WR_2,  -45.0}, {LA_WR_3, 60.0}, {LA_HA,   0.0}
//    {LA_WR_1,  0.0}, {LA_WR_2,   0.0}, {LA_WR_3,  0.0}, {LA_HA,  0.0}

};

const std::map<int, float> robot_ids_zero_pos_deg = {

    {WAIST_Y,   0.0},

    {RA_SH_1,  0.0}, {RA_SH_2, 10.0}, {RA_SH_3,20.0}, {RA_EL, 0.0},
    {RA_WR_1,  0.0}, {RA_WR_2,  0.0}, {RA_WR_3, 0.0}, {RA_HA, 0.0},

    {LA_SH_1,  0.0}, {LA_SH_2,-10.0}, {LA_SH_3,-20.0}, {LA_EL, 0.0},
    {LA_WR_1,  0.0}, {LA_WR_2,  0.0}, {LA_WR_3,  0.0}, {LA_HA, 0.0}

};

const std::map<int, float> robot_ids_up_pos_deg = {

    {WAIST_Y, 0.0},

    {RA_SH_1, 45.0}, {RA_SH_2, 10.0}, {RA_SH_3,50.0}, {RA_EL, 90.0},
    {RA_WR_1,  0.0}, {RA_WR_2, 60.0}, {RA_WR_3, 0.0}, {RA_HA,  0.0},

    {LA_SH_1,-45.0}, {LA_SH_2,-10.0}, {LA_SH_3,-50.0}, {LA_EL,-90.0},
    {LA_WR_1,  0.0}, {LA_WR_2,-60.0}, {LA_WR_3,  0.0}, {LA_HA,  0.0}

};

const std::map<int, float> robot_ids_extend_pos_deg = {

    {WAIST_Y,  0.0},

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
