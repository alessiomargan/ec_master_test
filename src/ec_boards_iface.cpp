#include <iit/ecat/advr/ec_boards_iface.h>
#include "iit/mc_tm4c/objectlist.h"

#include <math.h>
#include <pwd.h>


using namespace iit::ecat::advr;


Ec_Boards_ctrl::Ec_Boards_ctrl(const char * config_file) {

    // read conf file .....
    
    const char *homedir;
    
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }
    eth_if = std::string(config_file);

    //sync_cycle_time_ns = 1e6;     //   1ms
    //sync_cycle_time_ns = 100e6;     //   100ms
    sync_cycle_time_ns = 0;         //   no dc 
    sync_cycle_offset_ns = 500e6;   // 500ms

    std::string pipe_name = homedir; pipe_name+= "/get_param";
    get_param_pipe = new Write_XDDP_pipe(pipe_name, 16384);
    pipe_name = homedir; pipe_name+= "/set_param";
    set_param_pipe = new Read_XDDP_pipe(pipe_name, 16384);

}

Ec_Boards_ctrl::~Ec_Boards_ctrl() {

    iit::ecat::finalize();

    for (auto it = slaves.begin(); it != slaves.end(); it++) {
        delete it->second.get();
    }
}

int Ec_Boards_ctrl::init(void) {

    if ( iit::ecat::initialize(eth_if.c_str()) <= 0) {
        return 0;
    }

    factory_board();

    return 1;

}

void Ec_Boards_ctrl::factory_board(void) {

    //advr::TestESCTypes::pdo_tx  test_slave_1_tx_pdo;
    //advr::TestESC *             test_slave_1 = new advr::TestESC(ec_slave[1]);
    //slaves[1] = ESCPtr(test_slave_1);
    
    advr::McESC *               mc_slave = new advr::McESC(ec_slave[1]);
    slaves[1] = ESCPtr(mc_slave);

    //advr::McESC *               mc2_slave = new advr::McESC(ec_slave[3]);
    //slaves[3] = ESCPtr(mc2_slave);

    iit::ecat::setExpectedSlaves(slaves);


}

int Ec_Boards_ctrl::set_param(int slave_pos, int index, int subindex, int size, void *data) {

    return ec_SDOwrite(slave_pos, index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
}

int Ec_Boards_ctrl::get_param(int slave_pos, int index, int subindex, int *size, void *data) {

    return ec_SDOread(slave_pos, index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
}

void Ec_Boards_ctrl::configure_boards(void) {

    int wkc;
    int size;

    //short int controller_status = 2;
    //int wkc = ec_SDOwrite(2, 0x8001, 0x3, FALSE, sizeof(controller_status), &controller_status, EC_TIMEOUTRXM);
    //int wkc = set_param(2, 0x8001, 0x3, sizeof(controller_status), &controller_status);
    //if (wkc <= 0 ) {
    //    DPRINTF("fail sdo write\n");
    //}
    /*
    char fw_ver[16];
    int size = 8;
    wkc = get_param(2, 0x8001, 0x1, &size, fw_ver);
    if (wkc <= 0 ) {
        DPRINTF("fail sdo write\n");
    }
    DPRINTF("FW ver %s\n", fw_ver);
    */
    tDriveParameters tdrive;
    size = 4;
    wkc = get_param(1, 0x8000, 0x2, &size, &tdrive.TorGainP);
    if (wkc <= 0 ) { DPRINTF("fail sdo write\n"); }
    DPRINTF("TorGainP %d %f\n", size, &tdrive.TorGainP);


}


int Ec_Boards_ctrl::set_operative() {

    expected_wkc = iit::ecat::operational(&sync_cycle_time_ns, &sync_cycle_offset_ns);
    
    return expected_wkc;
}


int Ec_Boards_ctrl::recv_from_slaves() {

    /////////////////////////////////////////////
    // wait for cond_signal 
    // ecat_thread sync with DC
    int ret = iit::ecat::recv_from_slaves(&timing);
    if ( ret != 0 ) {
        DPRINTF("fail recv_from_slaves\n");
        return 0;
    }

    ///////////////////////////////////////////////////////////////////////////
    for (auto it = slaves.begin(); it != slaves.end(); it++) {
        
        McESC * s = dynamic_cast<McESC*>(it->second.get());
        if (s) {
            s->write_pdo_to_pipe();
        }
    }

    ///////////////////////////////////////////////////////////////////////////


    return 1;
}

int Ec_Boards_ctrl::send_to_slaves() {

    int wkc, retry = 2;

    for (auto it = slaves.begin(); it != slaves.end(); it++) {
        
        McESC * s = dynamic_cast<McESC*>(it->second.get());
        if (s) {
            s->read_pdo_from_pipe();
        }
    }
    //////////////
    //McESC * s = dynamic_cast<McESC*>(slaves[2].get());
    //advr::McESCTypes::pdo_tx    mc_tx_pdo;
    //static double time;
    //mc_tx_pdo.ts = get_time_ns();
    //mc_tx_pdo.direct_ref = 3000 * sinf(2*M_PI*time);
    //time += 0.001;
    //s->setTxPDO(mc_tx_pdo);
    ////////////////////

    wkc = iit::ecat::send_to_slaves();

    while ( wkc < expected_wkc && retry--) {
        DPRINTF("## wkc %d\n", wkc);
        wkc = iit::ecat::send_to_slaves();
    }

    return retry;

}

bool get_info(std::string token,int& sub_index, int& size)
{
    const objd * lookup_table = SDO8000;
    int table_size = lookup_table[0].value;
    for (int i=1;i<table_size;i++)
    {
        std::string temp=(char*)lookup_table[i].data;
        if (temp==token)
        {
            std::cout<<token<<" at "<<lookup_table[i].subindex<<" size:"<<lookup_table[i].bitlength<<std::endl;
            sub_index=lookup_table[i].subindex;
            size=lookup_table[i].bitlength;
            return true;
        }
    }
    return false;
}


void Ec_Boards_ctrl::lookup_read(std::string token, float* data )
{
    int sub_index=0;
    int size=0;
    get_info(token,sub_index,size);
    int wkc = ec_SDOread(1, 0x8000, sub_index, false, &size, data, EC_TIMEOUTRXM);
    if (wkc <= 0 ) { DPRINTF("fail sdo read\n"); }
}

int Ec_Boards_ctrl::handle_SDO(void) {

    char buffer[4096]; 
    int i = 0;
    ssize_t nbytes = 0;
    int wkc;
    char value[1024];
    void * pvalue = value;

    memset(buffer, 0, sizeof(buffer));

    // NON-BLOCKING read
    nbytes = set_param_pipe->read((void*)buffer, sizeof(buffer));

    if (nbytes > 0) {

        DPRINTF("read  %ld %s\n", nbytes, buffer);
        json_object * jObj = json_tokener_parse(buffer);
        DPRINTF( "JSON parse : %s\n", json_object_to_json_string(jObj));

        struct json_object_iterator it;
        struct json_object_iterator itEnd;
        it = json_object_iter_begin(jObj);
        itEnd = json_object_iter_end(jObj);
        while (!json_object_iter_equal(&it, &itEnd)) {
            std::string token=json_object_iter_peek_name(&it);
            json_object *jn;
            bool result=json_object_object_get_ex(jObj,token.c_str(),&jn);
            int size;
            int sub_index;
            get_info(token,sub_index,size);
            auto type=json_object_get_type(jn);

            DPRINTF( "type %s\n", json_type_to_name(type) );

            if (type==json_type_double)
            {
                float value = json_object_get_double(jn);
                wkc = set_param(1, 0x8000, sub_index, size/8, &value);
                DPRINTF( "wkc %d %f\n", wkc,value );
            } else if (type==json_type_int)
            {
                int value = json_object_get_int(jn);
                wkc = set_param(1, 0x8000, sub_index, size/8, &value);
                DPRINTF( "wkc %d %d\n", wkc,value );
            } else if (type==json_type_string)
            {
                std::string value=json_object_get_string(jn);
                DPRINTF( "wkc %d %s\n", wkc,value.c_str() );
            } else {

               DPRINTF( "Unknown type %s\n", json_type_to_name(type) );
            }
            

            json_object_iter_next(&it);
        }

    }

    /////////////////////////
    ///
    /////////////////////////
    tDriveParameters tdrive;
    json_serializer serializer;
#define params(x) #x,&tdrive.x
   lookup_read(params( TorGainP));
   lookup_read(params(TorGainI));
   lookup_read(params(TorGainD));
   lookup_read(params( TorGainFF));
   //lookup_read(params( Pos_I_lim));
   //lookup_read(params( Tor_I_lim));
   //lookup_read(params( Min_pos));
   //lookup_read(params(Max_pos));
   //lookup_read(params(Max_vel));
   //lookup_read(params(Max_tor));
   //lookup_read(params(Max_cur));
   lookup_read(params(Enc_offset));
   lookup_read(params(Enc_relative_offset));
   lookup_read(params( Phase_angle));

#undef params
    json_object * jObj = json_object_new_object();
    //void serializeToJson(tDriveParameters& tdrive, json_object* jObj)
    serializer.serializeToJson(tdrive,jObj);
    std::string json_str;
    json_str = json_object_to_json_string(jObj);
    json_str += "\n";


    nbytes = get_param_pipe->write((void*)json_str.c_str(), json_str.length());


}
