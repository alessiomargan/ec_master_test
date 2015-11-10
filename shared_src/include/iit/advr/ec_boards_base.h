/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)
   
*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_THREAD_BOARDS_BASE_H__
#define __EC_THREAD_BOARDS_BASE_H__

#include <iit/ecat/advr/ec_boards_iface.h>

#include <iit/advr/thread_util.h>
#include <iit/advr/pipes.h>
#include <iit/advr/trajectory.h>

/**
 */

		  
class Ec_Thread_Boards_base :
    public Thread_hook,
    public iit::ecat::advr::Ec_Boards_ctrl
{
public:
    
    Ec_Thread_Boards_base(const char * config_yaml) : Ec_Boards_ctrl(config_yaml) {}
    virtual ~Ec_Thread_Boards_base();

    bool init_OK() { return init_done; };

    virtual void th_init(void *);
    virtual void th_loop(void *);
    
    virtual int user_loop(void) = 0;

protected :
    
    virtual void init_preOP(void) = 0;
    virtual void init_OP(void) = 0;
    
    iit::ecat::stat_t  s_loop;
    uint64_t start_time, tNow, tPre;
    
    std::map<int,float> home;
    std::map<int,float> start_pos;

    void xddps_init(void);
    void xddps_loop(void);
    std::map<int,XDDP_pipe*> xddps;

    std::map<int, iit::ecat::advr::Motor*> 	motors;
    std::map<int, iit::ecat::advr::Ft6ESC*> 	fts;
    std::map<int, iit::ecat::advr::PowESC*> 	pows;
    std::map<int, iit::ecat::advr::PowComanESC*>powCmns;
    std::map<int, iit::ecat::advr::TestESC*> 	tests;

    bool go_there(const std::map<int, iit::ecat::advr::Motor*> motor_set,
		  const std::map<int,float> target_pos,
		  float eps);

    template <typename T>
    bool go_there(const std::map<int, iit::ecat::advr::Motor*> motor_set,
		  const std::map<int,advr::trajectory<T>*> spline_trj,
		  float eps);
private:
    
    bool init_done;
    
};


/**
 * NOTE this is a step reference !!!
 * LoPowerMotor (i.e. Coman) has a trajectory generator with max speed 0.5 rad/s
 * HiPowerMotor does NOT have it
 */
inline bool Ec_Thread_Boards_base::go_there(const std::map<int, iit::ecat::advr::Motor*> motor_set,
				            const std::map<int,float> target_pos,
				            float eps) {

    bool cond, all_true = true;
    float pos_ref, motor_err, link_err, motor_link_err;
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    std::vector<bool> truth_vect(motor_set.size()-1, true);
        
    for ( auto const& item : motor_set ) {
	slave_pos = item.first;
	moto =  item.second;
	
	// check in the target_pos map if the current slave_pos exist
	try { pos_ref = target_pos.at(slave_pos); }
	catch ( const std::out_of_range& oor ) { continue; }
	
	motor_pdo_rx = moto->getRxPDO();
	moto->set_posRef(pos_ref);
	
	link_err = fabs(motor_pdo_rx.link_pos  - pos_ref);
	motor_err = fabs(motor_pdo_rx.motor_pos - pos_ref);
	motor_link_err = fabs(motor_pdo_rx.motor_pos - motor_pdo_rx.link_pos);
	cond = ( link_err <= eps || motor_err <= eps );
	truth_vect.push_back( cond );
    
// 	if ( ! cond ) {
// 	    DPRINTF("%d %f %f{%f} %f{%f} {{%f}}\n",
// 		    pos2Rid(slave_pos), pos_ref,
// 		    motor_pdo_rx.link_pos, link_err,
// 		    motor_pdo_rx.motor_pos, motor_err,
// 		    motor_link_err);
// 	}
    }
    
    //DPRINTF("---\n");
    std::for_each(truth_vect.begin(),truth_vect.end(),[&](bool b){
	all_true &= b;
	//DPRINTF("%d ",b);
    });
    //DPRINTF("\n===\n");

    return all_true; 
}

template <typename T>
inline bool Ec_Thread_Boards_base::go_there(const std::map<int, iit::ecat::advr::Motor*> motor_set,
					    const std::map<int,advr::trajectory<T>*> spline_trj,
					    float eps) {

    bool cond, all_true = true;
    float pos_ref, motor_err, link_err, motor_link_err;
    int slave_pos;
    advr::trajectory<T> *trj;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    std::vector<bool> truth_vect(motor_set.size()-1, true);
        
    for ( auto const& item : motor_set ) {
	slave_pos = item.first;
	moto =  item.second;
	
	// test if the current slave_pos exist
	try { trj = spline_trj.at(slave_pos); }
	catch ( const std::out_of_range& oor ) { continue; }
	
	motor_pdo_rx = moto->getRxPDO();
	pos_ref = (float)(*trj)();
	moto->set_posRef(pos_ref);

	link_err = fabs(motor_pdo_rx.link_pos  - pos_ref);
	motor_err = fabs(motor_pdo_rx.motor_pos - pos_ref);
	motor_link_err = fabs(motor_pdo_rx.motor_pos - motor_pdo_rx.link_pos);
	cond = ( (link_err <= eps || motor_err <= eps) && trj->finish() );
	truth_vect.push_back( cond );

// 	if ( ! cond ) {
// 	    DPRINTF("%d %f %f{%f} %f{%f} {{%f}}\n",
// 		    pos2Rid(slave_pos), pos_ref,
// 		    motor_pdo_rx.link_pos, link_err,
// 		    motor_pdo_rx.motor_pos, motor_err,
// 		    motor_link_err);
// 	}
    }
    
    //DPRINTF("---\n");
    std::for_each(truth_vect.begin(),truth_vect.end(),[&](bool b){
	all_true &= b;
	//DPRINTF("%d ",b);
    });
    //DPRINTF("\n===\n");
    

    return all_true; 
}


#endif
