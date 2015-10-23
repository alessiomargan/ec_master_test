#include <stdio.h>

#include "iit/advr/pos_spline.h"
#include "iit/advr/spline.h"


int main(void) {

    std::vector<double> X(5), Y(5);
    X[0]=0.0; X[1]=4.0; X[2]=12.0; X[3]=18.0; X[4]=20.0;
    Y[0]=0.1; Y[1]=0.7; Y[2]=0.6; Y[3]=1.1; Y[4]=0.9;

    tk::spline s;
    s.set_points(X,Y);    // currently it is required that X is already sorted

    std::vector<Position> Pos(5);
    Pos[0].mPositionTime = 0.0;
    Pos[0].mLocation << 0.1, 0.0, 0.0;
    Pos[1].mPositionTime = 4.0;
    Pos[1].mLocation << 0.7, 0.0, 0.0;
    Pos[2].mPositionTime = 12.0;
    Pos[2].mLocation << 0.6, 0.0, 0.0;
    Pos[3].mPositionTime = 18.0;
    Pos[3].mLocation << 1.1, 0.0, 0.0;
    Pos[4].mPositionTime = 20.0;
    Pos[4].mLocation << 0.9, 0.0, 0.0;
    
    Trajectory trj;
    trj.set_points(Pos);
    
    double x = 20.0/100;
    
    for (int i=0; i < 100; i++) {
	printf("spline at %f is %f\t%f\n", x*i, s(x*i), trj(x*i).mLocation[0]);
    }
    return 0;
}
