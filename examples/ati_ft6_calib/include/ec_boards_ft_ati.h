/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_FT_ATI_H__
#define __EC_BOARDS_FT_ATI_H__

#include <iit/advr/ec_boards_base.h>
#include <iit/advr/ati_iface.h>

#include <boost/circular_buffer.hpp>
#include <boost/tokenizer.hpp>

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"


typedef struct {
    uint64_t    ts;
    float       ati[6];
    float       iit[6];
    float       dummy[6];
    
    void sprint ( char *buff, size_t size ) {
        snprintf ( buff, size, "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f", ts,
// big sensor
//                 -ati[0],ati[1],ati[2],-ati[3],ati[4],ati[5] );
//                 iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
// small sensor
                iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
                ati[0],ati[1],ati[2],ati[3],ati[4],ati[5]);
// cogimon foot
//                 iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
//                 -ati[1],ati[0],ati[2],-ati[4],ati[3],ati[5]);
    }
    void fprint ( FILE *fp ) {
        fprintf ( fp, "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", ts,
// big sensor
//                  iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
//                  -ati[0],ati[1],ati[2],-ati[3],ati[4],ati[5] );
// small sensor
                iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
                ati[0],ati[1],ati[2],ati[3],ati[4],ati[5]);
// cogimon foot
//                 iit[0],iit[1],iit[2],iit[3],iit[4],iit[5],
//                 -ati[1],ati[0],ati[2],-ati[4],ati[3],ati[5]);
    }
    
} sens_data_t ;



inline int load_matrix ( std::string filename, std::vector<std::vector<float>> &cal_matrix ) {

    std::string     line;
    std::fstream    file ( filename, std::ios::in );

    typedef boost::tokenizer<boost::char_separator<char>> Tokenizer;
    boost::char_separator<char> sep ( "," );

    if ( ! file ) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return -1;
    }

    while ( getline ( file, line ) ) {

        Tokenizer info ( line, sep ); // tokenize the line of data

        std::vector<float> values;

        for ( Tokenizer::iterator it = info.begin(); it != info.end(); ++it ) {
            // convert data into double value, and store
            values.push_back ( atof ( it->c_str() ) );
        }
        // store array of values
        cal_matrix.push_back ( values );
    }

    auto it = cal_matrix.begin();
    while ( it != cal_matrix.end() ) {
        std::vector<float> row ( *it );
        std::vector<float>::const_iterator rit = row.begin();
        while ( rit != row.end() ) {
            std::cout << ( *rit ) << "\t";
            rit ++;
        }
        std::cout << std::endl;
        it ++;
    }

    
    return 0;
}



class Ec_Boards_ft_ati : public Ec_Thread_Boards_base {
public:

    Ec_Boards_ft_ati ( const char * config_yaml );
    virtual ~Ec_Boards_ft_ati();

    template<class C>
    int user_input ( C &user_cmd );
    virtual int user_loop ( void );

private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );

    iit::ecat::advr::Ft6ESC* ft;
    iit::advr::Ati_Sens * ati;
    
    YAML::Node ati_config;
    
    boost::circular_buffer<sens_data_t> sens_log;
    iit::advr::ati_log_t   sample;
    sens_data_t sens_data;

    uint64_t    start;

    int serializeToXddp( const iit::advr::ati_log_t &ati_rx );
    XDDP_pipe   ftAtiOutXddp;
    std::string pb_str;
    
};


inline int Ec_Boards_ft_ati::serializeToXddp( const iit::advr::ati_log_t &ati ) {
         
    static iit::advr::FT_ati_rx pb_ati;
    //
    pb_ati.set_aforce_x (ati.ft[0]);
    pb_ati.set_aforce_y (ati.ft[1]);
    pb_ati.set_aforce_z (ati.ft[2]);
    pb_ati.set_atorque_x(ati.ft[3]);
    pb_ati.set_atorque_y(ati.ft[4]);
    pb_ati.set_atorque_z(ati.ft[5]);
    pb_ati.SerializeToString(&pb_str);
    auto msg_size = pb_str.length();
    auto nbytes = write ( ftAtiOutXddp.get_fd(), ( void* ) &msg_size, sizeof( msg_size ) );
    nbytes += write ( ftAtiOutXddp.get_fd(), ( void* ) pb_str.c_str() , pb_str.length() );
    
    return nbytes;

}


#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
