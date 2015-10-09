#include "Matrix.h"

MatrixClass::MatrixClass()
{
	(*this)(1,1) = 0.0;
	(*this)(1,2) = 0.0;
	(*this)(1,3) = 0.0;

	(*this)(2,1) = 0.0;
	(*this)(2,2) = 0.0;
	(*this)(2,3) = 0.0;

	(*this)(3,1) = 0.0;
	(*this)(3,2) = 0.0;
	(*this)(3,3) = 0.0;
}

MatrixClass MatrixClass::transpose()
{
	MatrixClass result;

	result(1,1) = (*this)(1,1);
	result(1,2) = (*this)(2,1);
	result(1,3) = (*this)(3,1);

	result(2,1) = (*this)(1,2);
	result(2,2) = (*this)(2,2);
	result(2,3) = (*this)(3,2);

	result(3,1) = (*this)(1,3);
	result(3,2) = (*this)(2,3);
	result(3,3) = (*this)(3,3);

	return result;
}

double MatrixClass::det()
{
	return ( (*this)(1,1)*((*this)(2,2)*(*this)(3,3)-(*this)(3,2)*(*this)(2,3))-(*this)(1,2)*((*this)(2,1)*(*this)(3,3)-(*this)(3,1)*(*this)(2,3))+(*this)(1,3)*((*this)(2,1)*(*this)(3,2)-(*this)(3,1)*(*this)(2,2)));
}

MatrixClass MatrixClass::inverse()
{
	MatrixClass result;
	double det;

	det= (*this)(1,1)*((*this)(2,2)*(*this)(3,3)-(*this)(3,2)*(*this)(2,3))-(*this)(1,2)*((*this)(2,1)*(*this)(3,3)-(*this)(3,1)*(*this)(2,3))+(*this)(1,3)*((*this)(2,1)*(*this)(3,2)-(*this)(3,1)*(*this)(2,2));

	result(1,1) =  1.0/det*((*this)(2,2)*(*this)(3,3)-(*this)(3,2)*(*this)(2,3));
	result(2,1) = -1.0/det*((*this)(2,1)*(*this)(3,3)-(*this)(3,1)*(*this)(2,3));
	result(3,1) =  1.0/det*((*this)(2,1)*(*this)(3,2)-(*this)(3,1)*(*this)(2,2));

	result(1,2) = -1.0/det*((*this)(1,2)*(*this)(3,3)-(*this)(3,2)*(*this)(1,3));
	result(2,2) =  1.0/det*((*this)(1,1)*(*this)(3,3)-(*this)(3,1)*(*this)(1,3));
	result(3,2) = -1.0/det*((*this)(1,1)*(*this)(3,2)-(*this)(3,1)*(*this)(1,2));

	result(1,3) =  1.0/det*((*this)(1,2)*(*this)(2,3)-(*this)(2,2)*(*this)(1,3));
	result(2,3) = -1.0/det*((*this)(1,1)*(*this)(2,3)-(*this)(2,1)*(*this)(1,3));
	result(3,3) =  1.0/det*((*this)(1,1)*(*this)(2,2)-(*this)(2,1)*(*this)(1,2));

	return result;
}
