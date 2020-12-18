// ModernRobotics.cpp : Defines the entry point for the application.
//

#include "FoundationsMotion.h"
#include "modern_robotics.h"

void coordinate_to_csv(string);
void Week3Quiz();
void Week4Quiz();

int main()
{
	RAND_SEED();
	
	Week4Quiz();

	return 0;
}





void Week4Quiz()
{
	using Eigen::MatrixXd;
	using Eigen::Matrix4d;
	using Eigen::Vector4d;
	using Eigen::Vector3d;
	using Eigen::VectorXd;
	MatrixXd Ts(4, 4), Ta(4, 4), Tb(4, 4);
	Ts = Matrix4d::Identity();
	Ta << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 1, 0, 0, 0, 1;
	Tb << 1, 0, 0, 0, 0, 0, 1, 2, 0, -1, 0, 0, 0, 0, 0, 1;
	// Question 1:
	Matrix4d Tsa = Ts * Ta;
	// Question 2:
	Matrix4d Tbs = mr::TransInv((Ts * Tb));
	// Question 3:
	Matrix4d Tab = mr::TransInv(Tsa) * (Ts * Tb);
	// Question 5:
	Vector4d pb(1, 2, 3, 1);
	Vector4d temp = (Ts * Tb) * pb;
	Vector3d ps(temp[0], temp[1], temp[2]);
	// Question 7:
	VectorXd Vs(6), Va(6);
	Vs << 3, 2, 1, -1, -2, -3;
	Va = mr::Adjoint(mr::TransInv(Tsa)) * Vs;
	// Question 8:
	VectorXd tempAngular(6), Theta_vec(6);
	tempAngular = mr::se3ToVec(mr::MatrixLog6(Tsa));
	Theta_vec = mr::AxisAng6(tempAngular);
	double Theta = Theta_vec[6];
	// Question 9:
	VectorXd St(6);
	St << 0, 1, 2, 3, 0, 0;
	Matrix4d MatExp_St = mr::MatrixExp6(mr::VecTose3(St));
	// Question 10:
	VectorXd Fb(6), Fs(6);
	Fb << 1, 0, 0, 2, 1, 0;
	Fs = mr::Adjoint(Tbs).transpose() * Fb;
	// Question 11:
	Matrix4d T; T << 0, -1, 0, 3, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1;
	Matrix4d T_exp = mr::TransInv(T);
	// Question 12:
	VectorXd V(6);
	V << 1, 0, 0, 0, 2, 3;
	Matrix4d Vse3 = mr::VecTose3(V);
	// Question 13:
	Vector3d s(1, 0, 0), p(0, 0, 2); int h = 1;
	VectorXd Screw(6);
	Screw = mr::ScrewToAxis(p, s, h);
	// Question 14:
	Matrix4d S_theta, MatExp; S_theta << 0, -1.5708, 0, 2.3562, 1.5708, 0, 0, -2.3562, 0, 0, 0, 1, 0, 0, 0, 0;
	MatExp = mr::MatrixExp6(S_theta);
	// Question 15:
	Matrix4d T_log = mr::MatrixLog6(T);
}


void Week3Quiz()
{
	using Eigen::Matrix3i;
	using Eigen::MatrixXi;
	using Eigen::Vector3i;
	using Eigen::Vector3d;
	using Eigen::Matrix3d;
	Matrix3i Rs = Eigen::Matrix3i::Identity();
	Matrix3i Ra, Rb;
	Ra << 0, 1, 0, 0, 0, 1, 1, 0, 0;
	Rb << 1, 0, 0, 0, 0, 1, 0, -1, 0;
	// Question 1:
	Matrix3i Rsa = Rs * Ra;
	// Question 2:
	Matrix3i Rsb = Rs * Rb;
	Matrix3i Rbs = Rsb.transpose();
	// Question 3:
	Matrix3i Rab = Rsa.transpose() * Rsb;
	// Question 5:
	Vector3i pb(1, 2, 3);
	Vector3i ps = Rsb * pb;
	// Question 7:
	Vector3i ws(3, 2, 1);
	Vector3i wa = Rsa.transpose() * ws;
	// Question 8:
	double Theta = mr::AxisAng3(mr::so3ToVec(mr::MatrixLog3(Rsa.cast<double>())))[3];
	// Question 9:
	Vector3d omega_Theta(1., 2., 0.);
	Matrix3d ExpWt = mr::MatrixExp3(mr::VecToso3(omega_Theta));
	// Question 10:
	Vector3d omega(1, 2, 0.5);
	Matrix3d SO3 = mr::VecToso3(omega);
	// Question 11:
	Matrix3d OmegaHat_Theta;
	OmegaHat_Theta << 0, 0.5, -1, -0.5, 0, 2, 1, -2, 0;
	Matrix3d MatExp = mr::MatrixExp3(OmegaHat_Theta);
	// Question 12:
	Matrix3d R;
	R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
	Matrix3d MatLog = mr::MatrixLog3(R);
}

void coordinate_to_csv(string namefile)
{
	if (namefile.empty())
		throw exception("fail name file");

	ofstream ofs{ namefile };

	if (!ofs.is_open())
		throw exception("fail open file");
	
	float rand_max = static_cast<float>(RAND_MAX);

	Eigen::Matrix<float, 3, 4> y; // matrix 3x4 of floats
	Eigen::Vector3i d{ RAND(-100, 100), RAND(-100, 100), RAND(-100, 100) }; // vector 3x1 of ints
	
	

	for (size_t i = 0; i < 3; ++i)
		for (size_t j = 0; j < 4; ++j)
			y(i, j) = static_cast<float>(rand() / rand_max);

	stringstream ss;

	for (size_t i = 0; i < 3; ++i) {
		ss << " " << std::setprecision(6) << y(i, 0) << ", " << y(i, 1) << ", " << y(i, 2) << ", " << y(i, 3) << ", " << d(i) << "\n";
		ofs << ss.str();
		stringstream().swap(ss);
	}

	ofs.close();
}