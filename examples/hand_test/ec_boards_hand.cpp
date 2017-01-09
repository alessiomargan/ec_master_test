#include <ec_boards_hand.h>
#include <iit/advr/coman_robot_id.h>
#include <iit/advr/walkman_robot_id.h>
#include <iit/advr/centauro_robot_id.h>

using namespace iit::ecat::advr;

#define MAX_FORCE_FINGER 5000

enum hand_status : int {
    IDLE    = 0,
    OPENING = 1,
    CLOSING = 2,
};


enum Robot_IDs : int {
    FING_1 = 1,
    FING_2 = 2,
    FING_3 = 3
};

std::map<int, float> hand_open_pos_rad = {
    {FING_1,   2.87},
    {FING_2,   1.45},
    {FING_3,   4.37}
};

std::map<int, float> hand_close_pos_rad = {
    {FING_1,   1.5},
    {FING_2,   3.4},
    {FING_3,   4.8}
};


Ec_Boards_hand::Ec_Boards_hand ( const char* config_yaml ) : Ec_Thread_Boards_base ( config_yaml ) {

    name = "EC_boards_sine";
    // do not go above ....
    period.period = {0,1};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy );
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!

    // open pipe ... xeno xddp or fifo
    inXddp.init ( "EC_board_input" );

}

Ec_Boards_hand::~Ec_Boards_hand() {
    std::cout << "~" << typeid ( this ).name() << std::endl;
}

void Ec_Boards_hand::init_preOP ( void ) {

    iit::ecat::advr::LpHandESC * moto;
    int slave_pos;
    float min_pos, max_pos, link_pos, motor_pos;

    std::vector<int> test_rid = std::initializer_list<int> {

        FING_1,FING_2,FING_3
        
    };
    

    // !!!
    get_esc_map_byclass ( fingers,  test_rid );

    for ( auto const& item : fingers ) {
        slave_pos = item.first;
        moto = item.second;
        moto->readSDO_byname ( "Min_pos", min_pos );
        moto->readSDO_byname ( "Max_pos", max_pos );
        moto->readSDO_byname ( "motor_pos", motor_pos );
        moto->readSDO_byname ( "link_pos", link_pos );
        start_pos[slave_pos] = motor_pos; 
        home[slave_pos] = hand_open_pos_rad[pos2Rid(slave_pos)];
        
        DPRINTF ( ">> Joint_id %d motor %f link %f start %f home %f\n",
                  pos2Rid ( slave_pos ), motor_pos, link_pos, start_pos[slave_pos], home[slave_pos]);
        
        //////////////////////////////////////////////////////////////////////////
        // start controller :
        // - read actual joint position and set as pos_ref  
        assert ( moto->start ( CTRL_SET_POS_MODE ) == EC_BOARD_OK );
    }

    
    for ( auto const& item : fingers ) {
        slave_pos = item.first;
        moto = item.second;
        while ( ! moto->move_to ( home[slave_pos], 0.005 ) ) {
            osal_usleep ( 1000 );
        }
    }

}

void Ec_Boards_hand::init_OP ( void ) {


}

template<class C>
int Ec_Boards_hand::user_input ( C &user_cmd ) {

    static int  bytes_cnt;
    int     bytes;

    if ( ( bytes = inXddp.xddp_read ( user_cmd ) ) <= 0 ) {
        return bytes;
    }

    bytes_cnt += bytes;
    DPRINTF ( ">> %d %d\n",bytes, bytes_cnt );
    //DPRINTF(">> %d\n",cmd.value);

    return bytes;
}

int Ec_Boards_hand::user_loop ( void ) {

    iit::ecat::advr::LpHandESC * finger;
    int slave_pos;
    char what = 0;
    static int hand_status;
    
    if ( user_input ( what ) <= 0) {
        return 0;
    }

    ///////////////////////////////////////////////////////////////////////////
    /*
     * echo o > /tmp/EC_board_input
     * echo c > /tmp/EC_board_input
     *
     */ 
    ///////////////////////////////////////////////////////////////////////////
    
    switch ( what ) {
        case 'o':
            DPRINTF ( ">> Open\n" );
            for ( auto const& item : fingers ) { 
                slave_pos = item.first;
                finger = item.second;
                finger->set_posRef(hand_open_pos_rad[pos2Rid(slave_pos)]);
            }
            hand_status = OPENING;
            break;
            
        case 'c':
            DPRINTF ( ">> Close\n" );
            for ( auto const& item : fingers ) { 
                slave_pos = item.first;
                finger = item.second;
                finger->set_posRef(hand_close_pos_rad[pos2Rid(slave_pos)]);
            }
            hand_status = CLOSING;
            break;
    }
    
    if ( hand_status == CLOSING ) {
    
        if ( check_force_finger() ) {
            // 
            //hand_status = 
        }
    }
    
    return 0;

}


int Ec_Boards_hand::check_force_finger( void ) {

    int contact = 0;
    McHandEscPdoTypes::pdo_rx hand_pdo_rx;
    McHandEscPdoTypes::pdo_tx hand_pdo_tx;
    iit::ecat::advr::LpHandESC * finger;
    
    
    // for each finger ....
    for ( auto const& item : fingers ) { 
        finger = item.second;
        hand_pdo_rx = finger->getRxPDO();
        // for each sensor ....
        if ( hand_pdo_rx.analog1 > MAX_FORCE_FINGER ||
             hand_pdo_rx.analog2 > MAX_FORCE_FINGER ||
             hand_pdo_rx.analog3 > MAX_FORCE_FINGER   ) {
        
            DPRINTF ( "[%d] %d %d %d\n", item.first, hand_pdo_rx.analog1, hand_pdo_rx.analog2, hand_pdo_rx.analog3 );
    
            //
            finger->set_posRef(hand_pdo_rx.link_pos);
            contact = 1;
        }
    } 
    
    return contact;
}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
