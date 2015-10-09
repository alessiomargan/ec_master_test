#ifndef _MATRIX_H// header guards
#define _MATRIX_H

class MatrixClass
{
	double matrix[9];

public:

	MatrixClass();

	double& operator()(int n, int m)
	{
		return matrix[3*(n-1)+(m-1)];// to return the element in matrix
	}

	MatrixClass operator* (MatrixClass  rhs)
	{
		MatrixClass result;

		result(1,1) = (*this)(1,1)*rhs(1,1)+(*this)(1,2)*rhs(2,1)+(*this)(1,3)*rhs(3,1);
		result(1,2) = (*this)(1,1)*rhs(1,2)+(*this)(1,2)*rhs(2,2)+(*this)(1,3)*rhs(3,2);
		result(1,3) = (*this)(1,1)*rhs(1,3)+(*this)(1,2)*rhs(2,3)+(*this)(1,3)*rhs(3,3);

		result(2,1) = (*this)(2,1)*rhs(1,1)+(*this)(2,2)*rhs(2,1)+(*this)(2,3)*rhs(3,1);
		result(2,2) = (*this)(2,1)*rhs(1,2)+(*this)(2,2)*rhs(2,2)+(*this)(2,3)*rhs(3,2);
		result(2,3) = (*this)(2,1)*rhs(1,3)+(*this)(2,2)*rhs(2,3)+(*this)(2,3)*rhs(3,3);

		result(3,1) = (*this)(3,1)*rhs(1,1)+(*this)(3,2)*rhs(2,1)+(*this)(3,3)*rhs(3,1);
		result(3,2) = (*this)(3,1)*rhs(1,2)+(*this)(3,2)*rhs(2,2)+(*this)(3,3)*rhs(3,2);
		result(3,3) = (*this)(3,1)*rhs(1,3)+(*this)(3,2)*rhs(2,3)+(*this)(3,3)*rhs(3,3);

		return result;
	}

	MatrixClass operator+ (MatrixClass  rhs)
	{
	MatrixClass result;

		result(1,1) = (*this)(1,1)+rhs(1,1);
		result(1,2) = (*this)(1,2)+rhs(1,2);
		result(1,3) = (*this)(1,3)+rhs(1,3);

		result(2,1) = (*this)(2,1)+rhs(2,1);
		result(2,2) = (*this)(2,2)+rhs(2,2);
		result(2,3) = (*this)(2,3)+rhs(2,3);

		result(3,1) = (*this)(3,1)+rhs(3,1);
		result(3,2) = (*this)(3,2)+rhs(3,2);
		result(3,3) = (*this)(3,3)+rhs(3,3);

		return result;
	}

	MatrixClass operator- (MatrixClass  rhs)
	{
	MatrixClass result;

		result(1,1) = (*this)(1,1)-rhs(1,1);
		result(1,2) = (*this)(1,2)-rhs(1,2);
		result(1,3) = (*this)(1,3)-rhs(1,3);

		result(2,1) = (*this)(2,1)-rhs(2,1);
		result(2,2) = (*this)(2,2)-rhs(2,2);
		result(2,3) = (*this)(2,3)-rhs(2,3);

		result(3,1) = (*this)(3,1)-rhs(3,1);
		result(3,2) = (*this)(3,2)-rhs(3,2);
		result(3,3) = (*this)(3,3)-rhs(3,3);

		return result;
	}

	MatrixClass transpose();
	double det();
	MatrixClass inverse();

};
#endif
