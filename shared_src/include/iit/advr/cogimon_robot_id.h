#ifndef __COGIMON_ROBOT_ID_H__
#define __COGIMON_ROBOT_ID_H__

namespace iit {
namespace ecat {
namespace advr {
namespace cogimon {


enum Robot_IDs {
    // waist
    WAIST_Y = 31,
    WAIST_R,
    // right leg
    RL_R    = 41,
    RL_P,
    RL_Y,
    RL_K,
    RL_T_DX,
    RL_T_SX,
    RL_FT   = 147,
    RL_FOOT = 148,
    // left leg
    LL_R = 51,
    LL_P,
    LL_Y,
    LL_K,
    LL_T_DX,
    LL_T_SX,
    LL_FT   = 157,
    LL_FOOT = 158,
};

typedef enum Robot_IDs Robot_IDs_t;

#define _MKS_(x) {#x,x}

const std::map<std::string, Robot_IDs_t > robot_ids_names = {
    // waist
    _MKS_(WAIST_Y),
    _MKS_(WAIST_R),
    // right leg
    _MKS_(RL_R),
    _MKS_(RL_P),
    _MKS_(RL_Y),
    _MKS_(RL_K),
    _MKS_(RL_T_DX),
    _MKS_(RL_T_SX),
    _MKS_(RL_FT),
    _MKS_(RL_FOOT),
    // left leg
    _MKS_(LL_R),
    _MKS_(LL_P),
    _MKS_(LL_Y),
    _MKS_(LL_K),
    _MKS_(LL_T_DX),
    _MKS_(LL_T_SX),
    _MKS_(LL_FT),
    _MKS_(LL_FOOT),
    
};


const std::vector<int> robot_head_ids = std::initializer_list<int> {
    // head
    //HEAD_R, HEAD_P
};

const std::vector<int> robot_waist_ids = std::initializer_list<int> {
    // waist
    WAIST_Y, WAIST_R
};

const std::vector<int> robot_right_leg_ids = std::initializer_list<int> {
    // right leg
    RL_R,
    RL_P,
    RL_Y,
    RL_K,
    RL_T_DX,
    RL_T_SX,
};

const std::vector<int> robot_left_leg_ids = std::initializer_list<int> {
    // left leg
    LL_R,
    LL_P,
    LL_Y,
    LL_K,
    LL_T_DX,
    LL_T_SX,
};

const std::vector<int> robot_right_arm_ids = std::initializer_list<int> {
    // right arm
};

const std::vector<int> robot_left_arm_ids = std::initializer_list<int> {
    // left arm
};

const std::vector<int> robot_hands_ids = std::initializer_list<int> {
    // hands
};

const std::vector<int> robot_fts_ids = std::initializer_list<int> {
    RL_FT, LL_FT,
    //RA_FT, LA_FT
};

const std::vector<int> robot_foot_ids = std::initializer_list<int> {
    RL_FOOT, LL_FOOT
};

const std::vector<int> robot_mcs_ids = std::initializer_list<int> {
    // head
    //HEAD_R, HEAD_P,
    // waist
    WAIST_Y, WAIST_R,
    // right leg
    RL_R, RL_P, RL_Y, RL_K, RL_T_DX, RL_T_SX,
    // left leg
    LL_R, LL_P, LL_Y, LL_K, LL_T_DX, LL_T_SX,
    // right arm
    //RA_SH_1, RA_SH_2, RA_SH_3, RA_EL, RA_WR_1, RA_WR_2, RA_WR_3, RA_HA,
    // left arm
    //LA_SH_1, LA_SH_2, LA_SH_3, LA_EL, LA_WR_1, LA_WR_2, LA_WR_3, LA_HA,

};


const std::map<const std::string, const std::vector<int>> robot_ids_group_names = {

    { "@robot_mcs",         robot_mcs_ids },
    { "@robot_hands",       robot_hands_ids },
    { "@robot_head",        robot_head_ids },
    { "@robot_waist",       robot_waist_ids },
    { "@robot_left_arm",    robot_left_arm_ids },
    { "@robot_right_arm",   robot_right_arm_ids },
    { "@robot_left_leg",    robot_left_leg_ids },
    { "@robot_right_leg",   robot_right_leg_ids },
        
};



const std::map<int, float> robot_ids_home_pos_deg = {

    {WAIST_Y, 0.0}, {WAIST_R, 0.0},
    // right leg
    {RL_R,   0.0}, {RL_P,    0.0}, {RL_Y,    0.0},
    {RL_K, -10.0}, {RL_T_DX, 0.0}, {RL_T_SX, 0.0},
    // left leg
    {LL_R,   0.0}, {LL_P,    0.0}, {LL_Y,    0.0},
    {LL_K,  10.0}, {LL_T_DX, 0.0}, {LL_T_SX, 0.0},

};



}
}
}
}

#endif

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
