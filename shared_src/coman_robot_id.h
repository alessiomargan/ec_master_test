namespace iit {
namespace ecat {
namespace advr {
namespace coman {


enum Robot_IDs 
{ 
    // waist
    WAIST_Y	= 1,
    WAIST_P	= 2,
    WAIST_R	= 3,

    // right leg
    RL_H_R	= 4,
    RL_H_Y	= 6,
    RL_H_P	= 7,
    RL_K 	= 8,
    RL_A_P	= 9,
    RL_A_R	= 10,
    RL_FT	= 34,
    
    // left leg
    LL_H_R	= 5,
    LL_H_Y	= 11,
    LL_H_P	= 12,
    LL_K	= 13,
    LL_A_P	= 14,
    LL_A_R	= 15,
    LL_FT	= 35,

#if 0    
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

#endif
    
}; 

}
}
}
}