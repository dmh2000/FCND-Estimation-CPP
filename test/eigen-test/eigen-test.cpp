#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
int main()
{
	Matrix3f x(3,3);
	Vector3f u(1.0f,2.0f,3.0f);

	std::cout << u << "\n";

	x.setOnes();
	std::cout << x << "\n";


	MatrixXf q;
    q = x * u;

	 std::cout <<  q << "\n";

	return 0;
}
