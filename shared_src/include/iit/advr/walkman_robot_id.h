#ifndef __WALKMAN_ROBOT_ID_H__
#define __WALKMAN_ROBOT_ID_H__

namespace iit {
namespace ecat {
namespace advr {
namespace walkman {


enum Robot_IDs {
    // neck
    HEAD_P = 1,
    HEAD_Y,
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
    //WAIST_P,
    WAIST_R,
    // right leg
    RL_H_R = 41,
    RL_H_Y,
    RL_H_P,
    RL_K,
    RL_A_P,
    RL_A_R,
    RL_FT,
    RL_FOOT,
    // left leg
    LL_H_R = 51,
    LL_H_Y,
    LL_H_P,
    LL_K,
    LL_A_P,
    LL_A_R,
    LL_FT,
    LL_FOOT,
    // test bench motors
    TEST_101 = 101,
    TEST_102 = 102,
    TEST_103 = 103,
};

typedef enum Robot_IDs Robot_IDs_t;

#define _MKS_(x) {#x,x}

const std::map<std::string, Robot_IDs_t > robot_ids_names = {
    // neck
    _MKS_(HEAD_P),
    _MKS_(HEAD_Y),
    // right arm
    _MKS_(RA_SH_1),
    _MKS_(RA_SH_2),
    _MKS_(RA_SH_3),
    _MKS_(RA_EL),
    _MKS_(RA_WR_1),
    _MKS_(RA_WR_2),
    _MKS_(RA_WR_3),
    _MKS_(RA_FT),
    _MKS_(RA_HA),
    // left arm
    _MKS_(LA_SH_1),
    _MKS_(LA_SH_2),
    _MKS_(LA_SH_3),
    _MKS_(LA_EL),
    _MKS_(LA_WR_1),
    _MKS_(LA_WR_2),
    _MKS_(LA_WR_3),
    _MKS_(LA_FT),
    _MKS_(LA_HA),
    // waist
    _MKS_(WAIST_Y),
    //_MKS_(WAIST_P),
    _MKS_(WAIST_R),
    // right leg
    _MKS_(RL_H_R),
    _MKS_(RL_H_Y),
    _MKS_(RL_H_P),
    _MKS_(RL_K),
    _MKS_(RL_A_P),
    _MKS_(RL_A_R),
    _MKS_(RL_FT),
    _MKS_(RL_FOOT),
    // left leg
    _MKS_(LL_H_R),
    _MKS_(LL_H_Y),
    _MKS_(LL_H_P),
    _MKS_(LL_K),
    _MKS_(LL_A_P),
    _MKS_(LL_A_R),
    _MKS_(LL_FT),
    _MKS_(LL_FOOT),
    // test bench motors
    _MKS_(TEST_101),
    _MKS_(TEST_102),
    _MKS_(TEST_103),
    
};


const std::vector<int> robot_head_ids = std::initializer_list<int> {
    // head
    HEAD_P, HEAD_Y
};

const std::vector<int> robot_waist_ids = std::initializer_list<int> {
    // waist
    WAIST_Y, WAIST_R
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
};

const std::vector<int> robot_hands_ids = std::initializer_list<int> {
    // hands
    RA_HA, LA_HA
};

const std::vector<int> robot_fts_ids = std::initializer_list<int> {
    RL_FT, LL_FT, RA_FT, LA_FT
};

const std::vector<int> robot_foot_ids = std::initializer_list<int> {
    RL_FOOT, LL_FOOT
};

const std::vector<int> robot_mcs_ids = std::initializer_list<int> {
    // head
    HEAD_P, HEAD_Y,
    // waist
    WAIST_Y, WAIST_R,
    // right leg
    RL_H_R, RL_H_Y, RL_H_P, RL_K, RL_A_P, RL_A_R,
    // left leg
    LL_H_R, LL_H_Y, LL_H_P, LL_K, LL_A_P, LL_A_R,
    // right arm
    RA_SH_1, RA_SH_2, RA_SH_3, RA_EL, RA_WR_1, RA_WR_2, RA_WR_3, RA_HA,
    // left arm
    LA_SH_1, LA_SH_2, LA_SH_3, LA_EL, LA_WR_1, LA_WR_2, LA_WR_3, LA_HA,

};

const std::vector<int> robot_mcs_upg_ids = std::initializer_list<int> {
    // torso roll & pitch
    WAIST_R,
    // right leg
    RL_H_R, RL_H_Y, RL_H_P, RL_K, RL_A_P, RL_A_R,
    // left leg
    LL_H_R, LL_H_Y, LL_H_P, LL_K, LL_A_P, LL_A_R,

};


const std::map<const std::string, const std::vector<int>> robot_ids_group_names = {

    { "@robot_mcs",         robot_mcs_ids },
    { "@robot_mcs_upg",     robot_mcs_upg_ids },
    { "@robot_hands",       robot_hands_ids },
    { "@robot_head",        robot_head_ids },
    { "@robot_waist",       robot_waist_ids },
    { "@robot_left_arm",    robot_left_arm_ids },
    { "@robot_right_arm",   robot_right_arm_ids },
    { "@robot_left_leg",    robot_left_leg_ids },
    { "@robot_right_leg",   robot_right_leg_ids },
        
};



const std::map<int, float> robot_ids_home_pos_deg = {

    {HEAD_P, 0.0}, {HEAD_Y, 0.0},
    
    {WAIST_Y, 0.0}, {WAIST_R, 0.0},
    // right leg
    {RL_H_R, 2.2}, {RL_H_Y,  0.15}, {RL_H_P, -17.0},
    {RL_K,  33.0}, {RL_A_P, 16.0},  {RL_A_R,  -2.0},
    // left leg
    {LL_H_R, -2.2}, {LL_H_Y, -0.15}, {LL_H_P, -17.0},
    {LL_K,   33.0}, {LL_A_P, 16.0},  {LL_A_R, 2.0},
    // right arm
    {RA_SH_1, 65.0}, {RA_SH_2, -70.0}, {RA_SH_3, 20.0}, {RA_EL, -150.0},
    {RA_WR_1, 0.0},  {RA_WR_2,   0.0}, {RA_WR_3, 35.0}, {RA_HA,    0.0},
    // left arm
    {LA_SH_1, 65.0}, {LA_SH_2, 70.0}, {LA_SH_3, -20.0}, {LA_EL, -150.0},
    {LA_WR_1, 0.0},  {LA_WR_2,  0.0}, {LA_WR_3, -35.0},  {LA_HA,   0.0}

};

const std::map<int, float> robot_ids_test_pos_deg = {

    {HEAD_P, 0.0}, {HEAD_Y, 0.0},
    
    {WAIST_Y, 0.0}, {WAIST_R, 0.0},

    {RL_H_R, -15}, {RL_H_Y, -35},   {RL_H_P, -80.0},
    {RL_K,  80.0}, {RL_A_P, -20.0}, {RL_A_R,  20.0},

    {LL_H_R,  15}, {LL_H_Y,  35},   {LL_H_P, -80.0},
    {LL_K,  80.0}, {LL_A_P,  -20.0}, {LL_A_R, -20.0},

    {RA_SH_1, 20.0}, {RA_SH_2, -20.0}, {RA_SH_3, -40.0}, {RA_EL, -50.0},
    {RA_WR_1,-20.0}, {RA_WR_2, -30.0}, {RA_WR_3, -40.0}, {RA_HA,   0.0},

    {LA_SH_1, 20.0}, {LA_SH_2,  20.0}, {LA_SH_3, 40.0}, {LA_EL,  -50.0},
    {LA_WR_1, 20.0}, {LA_WR_2, -30.0}, {LA_WR_3, 40.0}, {LA_HA,    0.0}

};

const std::map<int, float> robot_ids_arm_back_pos_deg = {

    {HEAD_P, 0.0}, {HEAD_Y, 0.0},
    
    {WAIST_Y, 0.0}, {WAIST_R, 0.0},
    // right leg
    {RL_H_R, -15}, {RL_H_Y,   0.2}, {RL_H_P, -90.0},
    {RL_K,  90.0}, {RL_A_P, -20.0}, {RL_A_R,  20.0},
    // left leg
    {LL_H_R,  15}, {LL_H_Y,  -0.2},  {LL_H_P, -90.0},
    {LL_K,  90.0}, {LL_A_P, -20.0}, {LL_A_R, -20.0},
    // right arm
    {RA_SH_1, 88.0}, {RA_SH_2, -70.0}, {RA_SH_3, 30.0}, {RA_EL, -5},
    {RA_WR_1,  0.0}, {RA_WR_2,   0.0}, {RA_WR_3,  0.0}, {RA_HA,  0.0},
    // left arm
    {LA_SH_1, 88.0}, {LA_SH_2,  70.0}, {LA_SH_3,-30.0}, {LA_EL, -5.0},
    {LA_WR_1,  0.0}, {LA_WR_2,   0.0}, {LA_WR_3,  0.0}, {LA_HA,  0.0}

};

const std::map<int, float> robot_ids_arm_front_pos_deg = {

    {HEAD_P, 0.0}, {HEAD_Y, 0.0},
    
    {WAIST_Y, 0.0}, {WAIST_R, 0.0},
    // right leg
    {RL_H_R, 2.2}, {RL_H_Y,  0.15}, {RL_H_P, -17.0},
    {RL_K,  33.0}, {RL_A_P, 16.0},  {RL_A_R,  -2.0},
    // left leg
    {LL_H_R, -2.2}, {LL_H_Y, -0.15}, {LL_H_P, -17.0},
    {LL_K,   33.0}, {LL_A_P, 16.0},  {LL_A_R, 2.0},
    // right arm
    {RA_SH_1, -20.0}, {RA_SH_2, -70.0}, {RA_SH_3, -30.0}, {RA_EL, -150.0},
    {RA_WR_1,  10.0}, {RA_WR_2,   0.0}, {RA_WR_3,  30.0}, {RA_HA,    0.0},
    // left arm
    {LA_SH_1, -20.0}, {LA_SH_2,  70.0}, {LA_SH_3,  30.0}, {LA_EL, -150.0},
    {LA_WR_1, -10.0}, {LA_WR_2,   0.0}, {LA_WR_3, -30.0}, {LA_HA,    0.0}

};


}
}
}
}

#endif

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
