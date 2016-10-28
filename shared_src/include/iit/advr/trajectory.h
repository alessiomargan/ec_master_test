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

inline double clamp(double d, double min, double max) {
  const double t = d < min ? min : d;
  return t > max ? max : t;
}

namespace advr {

        
class Trajectory {

public :

    virtual void set_points ( const std::vector<double> &x, const std::vector<double> &y ) = 0;
    
    virtual double get_value  ( double x, bool limits=true ) const = 0;

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
    
    virtual double end_point ( void ) {
        return _y.back();
    }
    
    virtual double end_time ( void ) {
        return _x.back();
    }
    
    virtual bool ended ( void ) {
        double x = (double)(iit::ecat::get_time_ns() - sT) / 1000000000 ;
        return ( x > _x.back() );
    }

protected:

    uint64_t sT;
    std::vector<double> _x, _y;
};

    
class Spline_trajectory : public Trajectory {

public :

    Spline_trajectory() {}
    Spline_trajectory( const std::vector<double> &x, const std::vector<double> &y ) {
        set_points( x, y );
    }

    void set_points ( const std::vector<double> &x, const std::vector<double> &y ) {

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

        s.set_points ( _x,_y, true );
        sT = iit::ecat::get_time_ns();
    };

    double get_value ( double x, bool limits=true ) const {
        if ( limits ) {
            if ( x < _x.front() ) return _y.front();
            if ( x > _x.back() ) return _y.back();
        }
        return s ( x );

    };
    

protected:

    tk::spline s;

};


class Smoother_trajectory : public Trajectory {

public :

    Smoother_trajectory() {}
    Smoother_trajectory( const std::vector<double> &x, const std::vector<double> &y ) {
        set_points( x, y );
    }

    virtual void set_points ( const std::vector<double> &x, const std::vector<double> &y ) {
    
        _x = x;
        _y = y;
        //std::cout << "zaq " << _x.size() << ' ' << _y.size() << '\n';
        
        assert ( _x.size() == _y.size() );
        
        sT = iit::ecat::get_time_ns();
        
    }
    
    double get_value ( double x, bool limits=true ) const {
        
        double fx, v;
        int i = 0;
        
        if ( x <= _x.front() )     { x = _x.front(); }
        else if ( x >= _x.back() ) { x = _x.back(); }

        // find i
        auto a = _x.begin();
        auto b = a+1;
        while ( b != _x.end() ) {
            //DPRINTF("Smoother_trajectory %f <= %f < %f\n", *a, x, *b );
            if ( *a <= x && x <= *b ) { break; }
            else { a++; b++; i++; }
        }

        x = clamp((x - _x[i])/(_x[i+1] - _x[i]), 0.0, 1.0);
        fx = x*x*x*(x*(x*6 - 15) + 10)*(_y[i+1]-_y[i]) + _y[i];
        //DPRINTF("Smoother_trajectory %d f(%f) = %f\n", i, x, fx );

        return fx;
    
    };

protected:


};


typedef std::shared_ptr<Trajectory> Trj_ptr;
typedef std::map<int, Trj_ptr> Trj_ptr_map;

inline void reset_trj ( Trj_ptr_map  &trjs ) {
    for ( auto item : trjs ) { item.second->start_time(); }
}


}

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
