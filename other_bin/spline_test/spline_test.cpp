#include <stdio.h>

#include "iit/advr/pos_spline.h"
#include "iit/advr/spline.h"
#include "iit/advr/trajectory.h"

#define SAMPLE_NUM 100

int main(void) {

    //std::vector<double> X = std::initializer_list<double> {   0,   5,  10,  15,  20 };
    //std::vector<double> Y = std::initializer_list<double> { 0.1, 0.7, 0.6, 1.1, 0.9 };
    std::vector<double> X = std::initializer_list<double> {   0,   1,   2 };
    std::vector<double> Y = std::initializer_list<double> { 0.0, 3.9, 1.0 };
    
    tk::spline s;
    s.set_points(X,Y);    // currently it is required that X is already sorted

    std::vector<Position> Pos(X.size());
    for (int i=0; i<X.size(); i++) {
	Pos[i].mPositionTime = X[i];
	Pos[i].mLocation << Y[i], 0.0, 0.0;
    	
    }	
    
    Trajectory trj;
    trj.set_points(Pos);
  
    advr::trajectory myt;
    myt.set_points(X,Y);
    
    
    double x = (X.back())/SAMPLE_NUM;
    
    for (int i=0; i < SAMPLE_NUM; i++) {
	
	printf("%f\t%f\t%f\t%f\t%f\n", x*i, s(x*i), trj(x*i).mLocation[0], myt(x*i), myt() );
	usleep(2000000/SAMPLE_NUM);
		
    }
    return 0;
}
