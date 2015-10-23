
#include "boost/chrono.hpp"
#include "Eigen/Splines"


class Position {
public:
  double mPositionTime; // Time of the position
  Eigen::Vector3d mLocation; // Location
};

typedef Eigen::Spline<double, 4> Spline4d;

class Trajectory {
    
public:
    Trajectory() {};
    void set_points(std::vector< Position > &);
    const Position operator() (const double x) const;
    
private:
    
    std::vector<Position> mPositionList;
};



void Trajectory::set_points(std::vector< Position >& posList)
{
    mPositionList = posList; 
//     for ( auto const& item : mPositionList ) {
// 	std::cout << item.mLocation << std::endl;
//     }
}


const Position Trajectory::operator() (const double positionTime) const{
  
    Position calcPosition;
    calcPosition.mPositionTime = positionTime;

    // Build up the knots along the spline. The first row is the msecs offset from the
    // first stored position. The second, third and fourth rows are X, Y, and Z
    // respectively.
    Eigen::MatrixXd points(4, mPositionList.size());
    for(int i = 0; i < mPositionList.size(); ++i) {
	// The spline position will be milliseconds.
	points(0, i) = mPositionList.at(i).mPositionTime - mPositionList.front().mPositionTime; // Time in msecs
	points(1, i) = mPositionList.at(i).mLocation(0); // X
	points(2, i) = mPositionList.at(i).mLocation(1); // Y
	points(3, i) = mPositionList.at(i).mLocation(2); // Z
    }
  
    // The degree of the interpolating spline needs to be one less than the number of points
    // that are fitted to the spline.
    const Spline4d spline = Eigen::SplineFitting<Spline4d>::Interpolate(points, mPositionList.size() - 1);
    const Eigen::Vector4d values = spline((positionTime - mPositionList.front().mPositionTime - points.row(0).minCoeff()) / (points.row(0).maxCoeff() - points.row(0).minCoeff()));
  
    // We already know the mPositionTime since that is how we
    // calculated the spline in the first place.
    calcPosition.mLocation(0) = values(1);
    calcPosition.mLocation(1) = values(2);
    calcPosition.mLocation(2) = values(3);
  
    return calcPosition;
}