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

#include <chrono>

namespace advr {
    
class trajectory {

public :
    
    trajectory() {}
    
    void set_points(const std::vector<double> &x, const std::vector<double> &y) {
    
	_x = x;
	_y = y;
	for (int i=0; i<_x.size(); i++ ) { _x[i] -= _x.front(); }
	s.set_points(_x,_y);
	sT = std::chrono::system_clock::now();
    };

    void start_time() { sT = std::chrono::system_clock::now(); }
    
    double operator() (double x) const {
	
	if ( x < _x.front() ) return _y.front();
	if ( x > _x.back() ) return _y.back();
	return s(x);
	
    };
    
    double operator() (void) const {
	
	std::chrono::duration<double> x = std::chrono::system_clock::now() - sT;
	//DPRINTF("%f\n", x.count());
	return trajectory::operator()(x.count());
   
    };
    
    double end_point(void) { return _y.back(); }
    
    bool finish(void) { 
	
	std::chrono::duration<double> x = std::chrono::system_clock::now() - sT;
	return ( x.count() > _x.back() );
	
    }
    
protected:
    
    std::chrono::time_point<std::chrono::system_clock> sT;
    std::vector<double> _x, _y;
    tk::spline s;
    
};

}

#endif