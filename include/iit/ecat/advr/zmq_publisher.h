/*

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)
   
*/

#ifndef __ZMQ_PUBLISHER_H__
#define __ZMQ_PUBLISHER_H__


#include <string>
#include <jsoncpp/json/json.h>

#include <thread_util.h>
#include <definitions.h>
#include <signals.h>
    

#include <zmq.hpp>
//#include <zmq.h>
//#include <zmq_utils.h>

#ifndef ZMQ_DONTWAIT
    #define ZMQ_DONTWAIT ZMQ_NOBLOCK
#endif
#if ZMQ_VERSION_MAJOR == 2
    #define zmq_msg_send(msg,sock,opt) zmq_send (sock, msg, opt)
    #define zmq_msg_recv(msg,sock,opt) zmq_recv (sock, msg, opt)
    #define zmq_ctx_destroy(context) zmq_term(context)
    #define ZMQ_POLL_MSEC 1000 // zmq_poll is usec
    #define ZMQ_SNDHWM ZMQ_HWM
    #define ZMQ_RCVHWM ZMQ_HWM
#elif ZMQ_VERSION_MAJOR == 3
    #define ZMQ_POLL_MSEC 1 // zmq_poll is msec
#endif

#include <broadcast_data.h>

extern zmq::context_t zmq_global_ctx;

///////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////

template<class T>
class Publisher {

public:

    Publisher(std::string pipe_name, std::string uri) {

        int opt_linger = 1;
#if ZMQ_VERSION_MAJOR == 2
        uint64_t opt_hwm = 1000;
#elif ZMQ_VERSION_MAJOR == 3
        int opt_hwm = 1000;
#endif
        _z = new zmq::socket_t(zmq_global_ctx, ZMQ_PUB);
        _z->setsockopt(ZMQ_LINGER, &opt_linger, sizeof(opt_linger));
        _z->setsockopt(ZMQ_SNDHWM, &opt_hwm, sizeof(opt_hwm));
        _z->bind(uri.c_str());
        printf ("publisher bind to %s\n",uri.c_str());
    }

    virtual ~Publisher() {
        std::cout << "~" << typeid(this).name() << std::endl;
        delete _z;
    }
    
    int data_size() = 0;
    virtual void publish(uint8_t * buffer) = 0;

protected:

    void publish_msg() {
        int cnt = 0;
        while (1) {
            try { 
                //printf("*** send %lu+%lu bytes\n", _msg_id.size() , _msg.size());
                _z->send(_msg_id, ZMQ_SNDMORE);
                _z->send(_msg);
                //printf("***\n");
                break;
            } catch (zmq::error_t& e) { // Interrupted system call
                cnt++;
                printf(">>> %d send ID ... catch %s\n", cnt, e.what());
            }
        }
    }
    
protected:
    
    zmq::socket_t * _z;
    //std::map<int, zmq::socket_t *>   _zpubs;
    zmq::message_t  _msg_id;
    zmq::message_t  _msg;

};

///////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////

#define JSON_EMITTER 1
#define TEXT_EMITTER 2
#define CSTRUCT_EMITTER 3


class Bc_Publisher: public Publisher {

private:
    ts_bc_data_t        ts_bc_data[MAX_DSP_BOARDS];

public:

    Bc_Publisher(std::map<int, std::string> uri_list): Publisher(uri_list) {}

    virtual int data_size() { return sizeof(ts_bc_data); }

    virtual void publish(uint8_t * buffer) {

        int ret = 0;
        bc_data_t * data = 0;
        memcpy(ts_bc_data, buffer, data_size());

#if 0
        ts_bc_data[MAX_DSP_BOARDS-1].raw_bc_data.bc_header._board_id = MAX_DSP_BOARDS;
        ts_bc_data[MAX_DSP_BOARDS-1].raw_bc_data.mc_bc_data.Position = 3000 * SIN10HZ;
        ts_bc_data[MAX_DSP_BOARDS-1].raw_bc_data.mc_bc_data.Velocity = 2000 * TRG1HZ;
        ts_bc_data[MAX_DSP_BOARDS-1].raw_bc_data.mc_bc_data.Torque = 1000 * TRG50HZ;
        ts_bc_data[MAX_DSP_BOARDS-1].raw_bc_data.mc_bc_data.PID_err = 4000 * SAW1HZ;
#endif

        for (int i=0; i<MAX_DSP_BOARDS; i++) {

            data = &ts_bc_data[i].raw_bc_data;
            if (data->bc_header._board_id > 0 && data->bc_header._board_id == i+1) {
                // valid bId ...
                for (std::map<int, zmq::socket_t *>::iterator it=_zpubs.begin(); it!=_zpubs.end(); it++) {
                    switch (it->first) {
                        case JSON_EMITTER:
                            ret = prepare_json_msg(data);
                            break;
                        case TEXT_EMITTER:
                            ret = prepare_text_msg(data);
                            break;
                        case CSTRUCT_EMITTER:
                            ret = prepare_cstruct_msg(data);
                            break;
                        default:
                            printf(">>> WARN %d emitter key not found\n", it->first);
                            ret = 0;
                    }
                    if ( ret > 0 ) {
                        prepare_msg_id(data->bc_header._board_id);
                        publish_msg(it->first);
                    } 
                }
            }
        }
    }

private:

    void prepare_msg_id(int bId) {
        
        std::string msg_id_prefix("board_");
        std::string text = msg_id_prefix + std::to_string((long long)bId);
        _msg_id.rebuild(text.length());
        memcpy((void*)_msg_id.data(),text.data(), text.length());

    }

    int prepare_json_msg(bc_data_t * data) {

        static Json::FastWriter    writer;
        static Json::Value         root;
        static bc_data_map_t       map_to_json;

        std::string json_string;
        unsigned char bc_data_board_type = data->bc_header._command;

        map_to_json.clear();

        bc_data_board_type = data->bc_header._command;
        switch ( bc_data_board_type ) {
            case BCAST_MC_DATA_PACKETS :
                data->mc_bc_data.to_map(map_to_json);
                break;
            case BCAST_FT_DATA_PACKETS :
                data->ft_bc_data.to_map(map_to_json);
                break;
            default:
                // empty ... 
                return 0;
        }

        for (bc_data_map_t::iterator it = map_to_json.begin(); it != map_to_json.end(); it++) {
            root[it->first] = it->second; 
        }

        json_string.append(writer.write(root));
        _msg.rebuild(json_string.length());
        memcpy((void*)_msg.data(), json_string.data(), json_string.length());

        return json_string.length();

    }

    int prepare_text_msg(bc_data_t * data) {

        char buffer[1024];
        unsigned char bc_data_board_type = data->bc_header._command;

        switch ( bc_data_board_type ) {
            case BCAST_MC_DATA_PACKETS :
                data->mc_bc_data.sprint(buffer, sizeof(buffer));
                break;
            case BCAST_FT_DATA_PACKETS :
                data->ft_bc_data.sprint(buffer, sizeof(buffer));
                break;
            default:
                // empty ...
                return 0;
        }

        std::string text(buffer);
        _msg.rebuild(text.length());
        memcpy((void*)_msg.data(), text.data(), text.length());

        return text.length();

    }

    int prepare_cstruct_msg(bc_data_t * data) {

        _msg.rebuild(sizeof(bc_data_t));
        memcpy((void*)_msg.data(), (void*)data , sizeof(bc_data_t));

        return sizeof(bc_data_t);

    }

};


///////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////


class Info_Publisher: public Publisher {

private:
    info_data_t     info_data;

public:

    Info_Publisher(std::map<int, std::string> uri_list): Publisher(uri_list) {

        std::string text("info");
        _msg_id.rebuild(text.length());
        memcpy((void*)_msg_id.data(),text.data(), text.length());

    }

    virtual int data_size() { return sizeof(info_data); }

    virtual void publish(uint8_t * buffer) {

        int ret = 0;
        memcpy((void*)&info_data, buffer, data_size());

        std::string text("Hello world !!");
        _msg.rebuild(text.length());
        memcpy((void*)_msg.data(), text.data(), text.length());

        publish_msg();

    } 

};



///////////////////////////////////////////////////////////////////////
///
//template <class T>
class Zmq_pub_thread : public Thread_hook {

public:
    Zmq_pub_thread(std::string pipe_name, Publisher * publisher);
    virtual ~Zmq_pub_thread();

    virtual void th_init(void *);
    virtual void th_loop(void *);

protected:

    Publisher  * publisher;
         
    std::string pipe_name;
    int xddp_sock;

    uint8_t buffer[8192];

};

#endif
