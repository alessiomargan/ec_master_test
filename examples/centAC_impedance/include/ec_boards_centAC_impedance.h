/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/
/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_CENTAC_IMPEDANCE_H__
#define __EC_BOARDS_CENTAC_IMPEDANCE_H__

#include <iit/advr/ec_boards_base.h>
#include <iit/advr/trajectory.h>
#include <protobuf/ec_boards_base_input.pb.h>

/**
 */

class EC_boards_centAC_impedance : public Ec_Thread_Boards_base {
public:

    EC_boards_centAC_impedance ( const char * config_yaml );
    virtual ~EC_boards_centAC_impedance();

    int xddp_input ( iit::advr::Ec_board_base_input & );
    int user_loop ( void );

    void tune_gains( std::vector<float> gains_incr );
    
private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );


    advr::Trj_ptr_map trj_start2home;
    advr::Trj_ptr_map trj_zero2up2extend2zero;
    advr::Trj_ptr_map trj_home2zero;

    std::map<int, iit::ecat::advr::Motor*> 	left_arm;;
    std::map<int, iit::ecat::advr::Motor*> 	right_arm;
    std::map<int, iit::ecat::advr::Motor*> 	waist;

    std::map<int, iit::ecat::advr::Motor*>  motors_ctrl_pos;
    std::map<int, iit::ecat::advr::Motor*>  motors_ctrl_imp;
    
protected:

    
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
