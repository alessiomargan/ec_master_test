#ifndef __COMAN_ROBOT_ID_H__
#define __COMAN_ROBOT_ID_H__

#include <initializer_list>
#include <algorithm>
#include <map>

namespace iit {
namespace ecat {
namespace advr {
namespace coman {


enum Robot_IDs : int {
    // waist
    WAIST_Y	= 1,
    WAIST_P	= 2,
    WAIST_R	= 3,

    // right leg
    RL_H_P	= 4,
    RL_H_R	= 6,
    RL_H_Y	= 7,
    RL_K 	= 8,
    RL_A_P	= 9,
    RL_A_R	= 10,
    RL_FT	= 34,

    // left leg
    LL_H_P	= 5,
    LL_H_R	= 11,
    LL_H_Y	= 12,
    LL_K	= 13,
    LL_A_P	= 14,
    LL_A_R	= 15,
    LL_FT	= 35,

    // right arm
    RA_SH_1	= 16,
    RA_SH_2	= 17,
    RA_SH_3	= 18,
    RA_EL	= 19,
    RA_WR_1	= 26,
    RA_WR_2	= 27,
    RA_WR_3	= 28,
    RA_FT	= 36,
    RA_HA	= 32,

    // left arm
    LA_SH_1	= 20,
    LA_SH_2	= 21,
    LA_SH_3	= 22,
    LA_EL	= 23,
    LA_WR_1	= 29,
    LA_WR_2	= 30,
    LA_WR_3	= 31,
    LA_FT	= 37,
    LA_HA	= 33,

};

const std::vector<int> robot_waist_ids = std::initializer_list<int> {
    // waist
    WAIST_Y, WAIST_P, WAIST_R
};

const std::vector<int> robot_right_leg_ids = std::initializer_list<int> {
    // right leg
    RL_H_R,
    RL_H_Y,
    RL_H_P,
    RL_K,
    RL_A_P,
    RL_A_R,
};

const std::vector<int> robot_left_leg_ids = std::initializer_list<int> {
    // left leg
    LL_H_R,
    LL_H_Y,
    LL_H_P,
    LL_K,
    LL_A_P,
    LL_A_R,
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
    RL_FT, LL_FT, RA_FT, LA_FT
};

const std::vector<int> robot_mcs_ids = std::initializer_list<int> {
    // waist
    WAIST_Y, WAIST_P, WAIST_R,
    // right leg
    RL_H_R, RL_H_Y, RL_H_P, RL_K, RL_A_P, RL_A_R,
    // left leg
    LL_H_R, LL_H_Y, LL_H_P, LL_K, LL_A_P, LL_A_R,
    // right arm
    RA_SH_1, RA_SH_2, RA_SH_3, RA_EL, RA_WR_1, RA_WR_2, RA_WR_3, RA_HA,
    // left arm
    LA_SH_1, LA_SH_2, LA_SH_3, LA_EL, LA_WR_1, LA_WR_2, LA_WR_3, LA_HA,

};

const std::map<int, float> robot_ids_home_pos_deg = {

    {WAIST_Y, 0.0}, {WAIST_P, 10.0}, {WAIST_R, 0.0},

    {RL_H_R, 0.0}, {RL_H_Y, 0.0},   {RL_H_P, -15.0},
    {RL_K, 30.0},  {RL_A_P, -15.0}, {RL_A_R, 0.0},

    {LL_H_R, 0.0}, {LL_H_Y, 0.0},   {LL_H_P, -15.0},
    {LL_K, 30.0},  {LL_A_P, -15.0}, {LL_A_R, 0.0},

    {RA_SH_1, 30.0}, {RA_SH_2, 70.0}, {RA_SH_3, 0.0}, {RA_EL, -90.0},
    {RA_WR_1, 0.0}, {RA_WR_2, 0.0}, {RA_WR_3, 0.0}, {RA_HA, 0.0},

    {LA_SH_1, 30.0}, {LA_SH_2,-70.0}, {LA_SH_3, 0.0}, {LA_EL, -90.0},
    {LA_WR_1, 0.0}, {LA_WR_2, 0.0}, {LA_WR_3, 0.0}, {LA_HA, 0.0}

};

const std::map<int, float> robot_ids_test_pos_deg = {

    {WAIST_Y, 0.0}, {WAIST_P, 0.0}, {WAIST_R, 0.0},

    {RL_H_R,-40.0}, {RL_H_Y, -30.0}, {RL_H_P, -70.0},
    {RL_K,   70.0}, {RL_A_P,  10.0}, {RL_A_R,  20.0},

    {LL_H_R, 40},   {LL_H_Y,  30.0}, {LL_H_P, -70.0},
    {LL_K,   70.0}, {LL_A_P,  10.0}, {LL_A_R, -20.0},

    {RA_SH_1,-40.0}, {RA_SH_2, -10.0}, {RA_SH_3, -20.0}, {RA_EL, -40.0},
    {RA_WR_1,-30.0}, {RA_WR_2,  15.0}, {RA_WR_3, -30.0}, {RA_HA,   0.0},

    {LA_SH_1,-40.0}, {LA_SH_2,  10.0}, {LA_SH_3, 20.0}, {LA_EL,  -40.0},
    {LA_WR_1, 30.0}, {LA_WR_2,  15.0}, {LA_WR_3, 30.0}, {LA_HA,    0.0}

};


}
}
}
}

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
