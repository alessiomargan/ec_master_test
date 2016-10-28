#include <stdio.h>

#include "iit/advr/pos_spline.h"
#include "iit/advr/spline.h"
#include "iit/advr/trajectory.h"

#define SAMPLE_NUM 100

int main ( void ) {

    //std::vector<double> X = std::initializer_list<double> {   0,   5,  10,  15,  20 };
    //std::vector<double> Y = std::initializer_list<double> { 0.1, 0.7, 0.6, 1.1, 0.9 };
    std::vector<double> X = std::initializer_list<double> {   0,  1,  2, 3 };
    std::vector<double> Y = std::initializer_list<double> {   0, 45,-45, 0 };
    std::vector<double> A = std::initializer_list<double> {   0,  10 };
    std::vector<double> B = std::initializer_list<double> { 0.0,   0.175 };

    tk::spline s;
    s.set_points ( X,Y ); // currently it is required that X is already sorted

    std::vector<Position> Pos ( X.size() );
    for ( int i=0; i<X.size(); i++ ) {
        Pos[i].mPositionTime = X[i];
        Pos[i].mLocation << Y[i], 0.0, 0.0;

    }

    Trajectory trj;
    trj.set_points ( Pos );

    advr::Spline_trajectory myt ( X,Y );
    advr::Spline_trajectory two_points ( A,B );
    advr::Smoother_trajectory smt ( X,Y );
    //advr::Smoother_trajectory smt ( A,B );

    double x = ( X.back() ) /SAMPLE_NUM;

#if 0
    for ( int i=0; i < SAMPLE_NUM; i++ ) {

        //printf("%f\t%f\t%f\t%f\t%f\t%f\n", x*i, s(x*i), trj(x*i).mLocation[0], myt(x*i), myt(), two_points() );
        //printf ( "%f\t%f\t%f\t%f\t%f\n", x*i, s ( x*i ), myt ( x*i ), myt(), two_points() );
        printf ( "%f\t%f\n", x*i, smt ( x*i ) );
        usleep ( 2000000/SAMPLE_NUM );

    }
#else
    for ( ;; ) {
        
        //if ( smt.ended() ) { smt.start_time(); }
        if ( smt.ended() ) { break; }
        printf ( "%f\n", smt() );
        usleep ( 10000 );
        
    }
#endif

    return 0;
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
