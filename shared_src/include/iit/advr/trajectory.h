/*
   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/
/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

//#include "iit/advr/pos_spline.h"
#include "iit/advr/spline.h"
#include "iit/ecat/utils.h"

#include <map>


namespace advr {


template <typename T>
class trajectory {

public :

//     trajectory(int size=128) {
// 	_x.reserve(size);
// 	_y.reserve(size);
//     }

    void set_points ( const std::vector<double> &x, const std::vector<double> &y, bool cubic_spline=true ) {

        _x = x;
        _y = y;
        for ( int i=0; i<_x.size(); i++ ) {
            _x[i] -= _x.front();
        }

        assert ( _x.size() == _y.size() );
        assert ( _y.size() > 1 );
        if ( _y.size() == 2 ) {

            _y.resize ( 3,_y[1] );
            _y[1] = ( _y[0]+_y[1] ) /2;

            _x.resize ( 3,_x[1] );
            _x[1] = _x[1]/2;
        }

        //for (int i=0;i<_x.size();i++)
        //    std::cout << ' ' << _x[i] << ' ' << _y[i] << '\n';

        t.set_points ( _x,_y,cubic_spline );
        sT = iit::ecat::get_time_ns();
    };

    double get_value ( double x, bool limits=true ) const {

        if ( limits ) {
            if ( x < _x.front() ) return _y.front();
            if ( x > _x.back() ) return _y.back();
        }
        return t ( x );

    };

    double operator() ( double x ) const {

        return get_value ( x );

    };

    double operator() ( void ) const {

        double x = (double)(iit::ecat::get_time_ns() - sT) / 1000000000 ;
        return get_value ( x );

    };

    void start_time() {
        sT = iit::ecat::get_time_ns();
    }
    
    void get_start_time ( uint64_t &start ) {
        start = sT;
    }

    double end_point ( void ) {
        return _y.back();
    }
    double end_time ( void ) {
        return _x.back();
    }

    bool finish ( void ) {

        double x = (double)(iit::ecat::get_time_ns() - sT) / 1000000000 ;
        return ( x > _x.back() );

    }

protected:

    uint64_t sT;
    std::vector<double> _x, _y;
    T t;

};

template<typename T> using Trj = trajectory<T>;
typedef Trj<tk::spline> Spline_Trj;
//typedef std::map<int,Spline_Trj*> Spline_ptr_map;
typedef std::map<int,Spline_Trj> Spline_map;

inline void reset_spline_trj ( Spline_map  &spls ) {
    //for ( auto item : spls ) { item.second.start_time(); }
    for ( auto it=spls.begin(); it!=spls.end(); ++it ) {
        it->second.start_time();
    }
}

}

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
