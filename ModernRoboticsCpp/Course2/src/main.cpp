// ModernRobotics.cpp : Defines the entry point for the application.
//


#include "PCH.h"



void Week1Quiz();
void Week2Quiz();
void Week3Quiz();
void Week3Project();

int main()
{
	RAND_SEED();
	Week3Project();
	

	return 0;
}




void Week1Quiz()
{
	cout << std::setprecision(3);
	// Question 1:
	double f = sqrt(3);
	double L = 1.;
	Matrix4d M;
	M << 1, 0, 0, (L + f * L + L), 0, 1, 0, 0, 0, 0, 1, (-L + f * L + 2 * L), 0, 0, 0, 1;
	
	// Question 2:
	MatrixXd Slist;
	Slist =  (MatrixXd(6,6) << 0, 0, 1, 0, -L, 0, 0, 1, 0, 0, 0, L, 0, 1, 0, L, 0, (L + f * L),
			      0, 1, 0, (-f * L + L), 0, (2 * L + f * L), 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, (-f * L - 2 * L), 0).finished().transpose();

	// Question 3:
	MatrixXd Blist;
	Blist = (MatrixXd(6, 6) << 0, 0, 1, 0, (L + f * L), 0, 0, 1, 0, (f * L + L), 0, (-L - f * L),
									0, 1, 0, (f * L + 2 * L), 0, -L, 0, 1, 0, 2 * L, 0, 0,
									0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0).finished().transpose();

	// Question 4:
	VectorXd Theta(6);
	Theta << -M_PI / 2, M_PI / 2, M_PI / 3, -M_PI / 4, 1, M_PI / 6;	
	Matrix4d T = mr::FKinSpace(M, Slist, Theta);

	// Question 5:
	T = mr::FKinBody(M, Blist, Theta);
}


void Week2Quiz()
{
	cout << std::setprecision(3);
	cout << std::fixed;
	int N = 1, L = 1;
	// Question 1:
	VectorXd Fs(6);
	Fs << 0, 0, 0, ((double)N * 2), 0, 0;
	Vector3d Theta(0, M_PI / 4, 0);
	MatrixXd ScrewAxesS(3, 6);
	ScrewAxesS << 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, -L, 0, 0, 0, 1, 0, -(2 * L), 0;
	MatrixXd Js(6, 6);
	Js = mr::JacobianSpace(ScrewAxesS.transpose(), Theta);
	Vector3d tau = Js.transpose() * Fs;

	// Question 2:
	Vector4d Theta2(0, 0, M_PI / 2, -M_PI / 2);
	Vector4d& t = Theta2;
	MatrixXd Jb;
	Jb = (MatrixXd(6, 4) << 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
							sin(t[3]) + sin(t[2] + t[3]) + sin(t[1] + t[2] + t[3]), sin(t[3]) + sin(t[2] + t[3]), sin(t[3]), 0,
							L + cos(t[3]) + cos(t[2] + t[3]) + cos(t[1] + t[2] + t[3]), L + cos(t[3]) + cos(t[2] + t[3]), L + cos(t[3]), L,
							0, 0, 0, 1).finished();
	VectorXd Fb = (VectorXd(6) << 0, 0, 10, 10, 10, 0).finished();
	Vector4d tau2 = Jb.transpose() * Fb;
	
	// Question 3:
	Vector3d Theta3(M_PI / 2, M_PI / 2, 1);
	VectorXd S1(6), S2(6), S3(6);
	S1 << 0, 0, 1, 0, 0, 0;
	S2 << 1, 0, 0, 0, 2, 0;
	S3 << 0, 0, 0, 0, 1, 0;
	MatrixXd Slist(3, 6);
	Slist.row(0) << S1.transpose(); 
	Slist.row(1) << S2.transpose();
	Slist.row(2) << S3.transpose();
	MatrixXd JacobiS = mr::JacobianSpace(Slist.transpose(), Theta3);
	
	// Question 4:	
	VectorXd B1(6), B2(6), B3(6);
	B1 << 0, 1, 0, 3, 0, 0;
	B2 << -1, 0, 0, 0, 3, 0;
	B3 << 0, 0, 0, 0, 0, 1;
	MatrixXd Blist(3, 6);
	Blist.row(0) << B1.transpose();
	Blist.row(1) << B2.transpose();
	Blist.row(2) << B3.transpose();	
	MatrixXd JacobiB = mr::JacobianBody(Blist.transpose(), Theta3);
	
	// Question 5:
	MatrixXd JacobianBody(6, 7);
	JacobianBody << 0, -1, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1,
					-0.105, 0, 0.006, -0.045, 0, 0.006, 0, -0.889, 0.006, 0, -0.844, 0.006, 0, 0, 0, -0.105, 0.889, 0, 0, 0, 0;
	MatrixXd Jv(3, 7);
	Jv << JacobianBody.row(3), JacobianBody.row(4), JacobianBody.row(5);
	MatrixXd A = Jv * Jv.transpose();
	EigenSolver<MatrixXd> EigM(A);
	VectorXcd::Index maxIndex;
	EigM.eigenvalues().real().maxCoeff(&maxIndex);
	VectorXd unitEig = EigM.eigenvectors().real().col(maxIndex);

	// Question 6:
	double len_unitEig = sqrt(EigM.eigenvalues().real()[maxIndex]);
}



static Vector2d func(const Vector2d& args)
{
	return Vector2d(args[0] * args[0] - 9, args[1] * args[1] - 4);
}

static Matrix2d derivate_func(const Vector2d& args)
{
	return (Matrix2d() << 2 * args[0], 0, 0, 2 * args[1]).finished();
}

void Week3Quiz()
{
	// Question 1:
	int n = 2, i = 0;
	Vector2d guess(1, 1);
	for (; i < n; ++i)
		guess = guess - (derivate_func(guess).inverse() * func(guess));
	
	// Question 2:
	int L = 1;
	Matrix4d Tsd = (Matrix4d() << -0.585, -0.811, 0, 0.076, 0.811, -0.585, 0, 2.608, 0, 0, 1, 0, 0, 0, 0, 1).finished();
	VectorXd Theta_guess = (VectorXd(3) << M_PI/4, M_PI/4, M_PI / 4).finished();
	double Ew = 0.001, Ev = 0.0001;
	Matrix4d M = (Matrix4d() << 1, 0, 0, (double)3 * L, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1).finished();
	VectorXd S_theta1 = (VectorXd(6) << 0, 0, 1, 0, 0, 0).finished();
	VectorXd S_theta2 = (VectorXd(6) << 0, 0, 1, 0, -L, 0).finished();
	VectorXd S_theta3 = (VectorXd(6) << 0, 0, 1, 0, -(2 * L), 0).finished();
	MatrixXd Slist(3, 6);
	Slist.row(0) << S_theta1.transpose();
	Slist.row(1) << S_theta2.transpose();
	Slist.row(2) << S_theta3.transpose();
	bool right = mr::IKinSpace(Slist.transpose(), M, Tsd, Theta_guess, Ew, Ev);

	if (right)
		cout << Theta_guess;
}


void Week3Project()
{
	// TODO

}