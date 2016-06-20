#ifndef __WALKMAN_ROBOT_ID_H__
#define __WALKMAN_ROBOT_ID_H__

namespace iit {
namespace ecat {
namespace advr {
namespace walkman {


enum Robot_IDs {
    // neck
    HEAD_R = 1,
    HEAD_P,

    // right arm
    RA_SH_1 = 11,
    RA_SH_2,
    RA_SH_3,
    RA_EL,
    RA_WR_1,
    RA_WR_2,
    RA_WR_3,
    RA_FT,
    RA_HA,

    // left arm
    LA_SH_1 = 21,
    LA_SH_2,
    LA_SH_3,
    LA_EL,
    LA_WR_1,
    LA_WR_2,
    LA_WR_3,
    LA_FT,
    LA_HA,

    // waist
    WAIST_Y = 31,
    WAIST_R,
    WAIST_P,

    // right leg
    RL_H_R = 41,
    RL_H_Y,
    RL_H_P,
    RL_K,
    RL_A_P,
    RL_A_R,
    RL_FT,

    // left leg
    LL_H_R = 51,
    LL_H_Y,
    LL_H_P,
    LL_K,
    LL_A_P,
    LL_A_R,
    LL_FT,
};

const std::vector<int> robot_waist_ids = std::initializer_list<int> {
    // waist
    WAIST_Y, WAIST_R, WAIST_P
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

std::map<int, float> robot_ids_home_pos_deg = {

    {WAIST_Y, 0.0}, {WAIST_P, 0.0}, {WAIST_R, 0.0},

    {RL_H_R, 2.2}, {RL_H_Y, 0.15}, {RL_H_P, -17.0}, {RL_K, 33.0}, {RL_A_P, 16.0}, {RL_A_R, -2.0},

    {LL_H_R, -2.2}, {LL_H_Y, -0.15}, {LL_H_P, -17.0}, {LL_K, 33.0}, {LL_A_P, 16.0}, {LL_A_R, 2.0},

    {RA_SH_1, 60.0}, {RA_SH_2, -70.0}, {RA_SH_3, 20.0}, {RA_EL, -110.0}, {RA_WR_1, 0.0}, {RA_WR_2, -30.0}, {RA_WR_3, 0.0}, {RA_HA, 0.0},

    {LA_SH_1, 60.0}, {LA_SH_2, 70.0}, {LA_SH_3, -20.0}, {LA_EL, -110.0}, {LA_WR_1, 0.0}, {LA_WR_2, -30.0}, {LA_WR_3, 0.0}, {LA_HA, 0.0}

};

}
}
}
}

#endif

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
