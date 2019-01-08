/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/
/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_BOARDS_CENTAC_URDF_H__
#define __EC_BOARDS_CENTAC_URDF_H__

#include <fstream>

#include <iit/advr/ec_boards_base.h>
#include <iit/advr/trajectory.h>

#include <rbdl/rbdl.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
        #error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

#include <upper_body_ik/ik_demo_class.h>

namespace rbd = RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

/**
 */

class URDF_adapter {

public:

    URDF_adapter() {}
    virtual ~URDF_adapter() { delete model; }
    
protected:

    void init_urdf ( std::string urdf_config_yaml ) {
        
        rbdl_check_api_version (RBDL_API_VERSION);
        model = new rbd::Model();
        const YAML::Node config = YAML::LoadFile ( urdf_config_yaml );
        std::string urdf_file = config["urdf_file"].as<std::string>();
        if (!rbd::Addons::URDFReadFromFile ( urdf_file.c_str(), model, true, true)) {
            std::cerr << "Error loading model " << std::endl;
            abort();
        }
        
        std::ifstream model_file (urdf_file);
        if (!model_file) {
            std::cerr << "Error opening file '" << urdf_file << "'." << std::endl;
            abort();
        }
        
        // reserve memory for the contents of the file
        model_file.seekg(0, std::ios::end);
        model_xml_string.reserve(model_file.tellg());
        model_file.seekg(0, std::ios::beg);
        model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());
        model_file.close();
        
        DPRINTF("RDB::model DOF count %d\n", model->dof_count);
        
        Q = VectorNd::Zero (model->dof_count);
        q_rbdl = VectorNd::Zero (model->dof_count);
        QDot = VectorNd::Zero (model->dof_count);
        Tau = VectorNd::Zero (model->dof_count);
        QDDot = VectorNd::Zero (model->dof_count);
        G = MatrixNd::Zero ( 6, model->dof_count + 6);
        Jack = MatrixNd::Zero ( 3, model->dof_count);
        
        try {
            urdf_joints_map = config["urdf_joints_map"].as<std::map<std::string,int>>();
            for ( auto const& item : urdf_joints_map ) {
                int id, q_id, rid;
                const char * body_name = item.first.c_str();
                id = model->GetBodyId(body_name);
                q_id = model->mJoints[id].q_index;
                rid = item.second;
                rid2urdf[rid] = q_id;
                urdf2rid[q_id] = rid;
                std::cout << body_name << ": " << id << " " << q_id << " " << rid << std::endl;
            }
        } catch ( YAML::Exception &e ) {
            DPRINTF ( "Catch Exception in %s ... %s\n", __PRETTY_FUNCTION__, e.what() );
        }

    }

    std::map<std::string,int>   urdf_joints_map;
    
    rbd::Model * model;    
    VectorNd Q;
    VectorNd q_rbdl;
    VectorNd QDot;
    VectorNd QDDot;
    VectorNd Tau;
    MatrixNd G;
    MatrixNd Jack;
    
    int rid2Urdf ( int rId ) {
        //return rid2urdf.find ( rId ) != rid2urdf.end() ? rid2urdf[rId] : 0;
        try { return rid2urdf.at(rId); } catch ( std::out_of_range &e ) {
            throw iit::ecat::advr::EcBoardsError( iit::ecat::advr::EC_WRP_NOK, std::string("wrong robot Id ")+std::to_string(rId) );
        }   
    }
    int urdf2Rid ( int uId ) {
        //return urdf2rid.find ( uId ) != urdf2rid.end() ? urdf2rid[uId] : 0;
        try { return urdf2rid.at(uId); } catch ( std::out_of_range &e ) {
            throw iit::ecat::advr::EcBoardsError( iit::ecat::advr::EC_WRP_NOK, std::string("wrong urdf Id ")+std::to_string(uId) );
        }   
    }
    
    std::vector<int> get_urdf_rId() {
        std::vector<int> rIds;
        for ( auto const& item : urdf_joints_map ) { rIds.push_back(item.second); }
        return rIds;
    } 

    std::string model_xml_string;
        
private:
    
    std::map<int, int> rid2urdf, urdf2rid;

};



class EC_boards_centAC_urdf : public Ec_Thread_Boards_base, URDF_adapter {
public:


    EC_boards_centAC_urdf ( const char * config_yaml );
    virtual ~EC_boards_centAC_urdf();

    template<class C>
    int xddp_input ( C &user_cmd );
    int user_loop ( void );

private :

    virtual void init_preOP ( void );
    virtual void init_OP ( void );

    XDDP_pipe jsInXddp, navInXddp, termInXddp;
    

    advr::Trj_ptr_map spline_start2home;

    std::map<int, iit::ecat::advr::Motor*> 	left_arm;;
    std::map<int, iit::ecat::advr::Motor*> 	right_arm;
    std::map<int, iit::ecat::advr::Motor*> 	waist;

    std::map<int, iit::ecat::advr::Motor*>  motors_to_start;
    
    UpperbodyDemoClass * demo_class;
    UpperbodyDemoClass::TrajectoryParameters left_arm_trj;
    UpperbodyDemoClass::TrajectoryParameters right_arm_trj;
    // store reference to feed set_pos
    std::map<std::string, double> q_map;
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
