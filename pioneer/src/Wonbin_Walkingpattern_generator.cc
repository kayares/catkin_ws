#include "Wonbin_Walkingpattern_generator.hpp"
using namespace Eigen;
using namespace std;
#define deg2rad		0.017453292519943
#define rad2deg		57.295779513082323
#define PI			3.141592653589793

Wonbin::Com::Com()
{
	walkfreq = 1.48114;
	walktime = 1 / walkfreq;
	stride = 0.1;
	freq = 500;
	del_t = 1 / freq;
	z_c = 1.2 * 0.30583;
	g = 9.81;
	T_prev = 1.5;
	NL = T_prev * freq;
	A << 1,   del_t,   del_t* del_t / 2,
		 0,     1,         del_t,
		 0,     0,           1;
	B << del_t * del_t * del_t / 6, del_t* del_t / 2, del_t;
	C << 1, 0, -z_c / g;
	Qe = 1;
	Qx = Matrix3d::Zero();
	Q_p = Matrix4d::Zero();
	Q_p << Qe, 0, 0, 0,
		0, Qx(0, 0), Qx(0, 1), Qx(0, 2),
		0, Qx(1, 0), Qx(1, 1), Qx(1, 2),
		0, Qx(2, 0), Qx(2, 1), Qx(2, 2);
	R << pow(10, -6);
	I_p << 1, 0, 0, 0;

	B_p.row(0) = C * B;
	B_p.row(1) = B.row(0);
	B_p.row(2) = B.row(1);
	B_p.row(3) = B.row(2);

	F_p.row(0) = C * A;
	F_p.row(1) = A.row(0);
	F_p.row(2) = A.row(1);
	F_p.row(3) = A.row(2);

	A_p = Matrix4d::Identity();
	A_p.block<4, 3>(0, 1) = F_p;

	K_p << 198.633829737069,	19628.3822432368,	3818.23902165455,	4.21279595103775,
		19628.3822432368,	1989741.22570983,	387263.130714345,	466.908230988326,
		3818.23902165455,	387263.130714345,	75375.0546442415,	91.2815487187198,
		4.21279595103775,	466.908230988326,	91.2815487187198,	0.190908542883096;

	Gi = (R + B_p.transpose() * K_p * B_p).inverse() * B_p.transpose() * K_p * I_p;
	Gx = (R + B_p.transpose() * K_p * B_p).inverse() * B_p.transpose() * K_p * F_p;
	Ac_p = A_p - B_p * (R + B_p.transpose() * K_p * B_p).inverse() * B_p.transpose() * K_p * A_p;
};

MatrixXd Wonbin::Com::PreviewGd()
{
	MatrixXd Gd(NL, 1);
	for (int l = 0; l < NL; l++) {

		Matrix4d temp = Ac_p.transpose();
		for (int i = 1; i < l; i++) {
			temp = temp * Ac_p.transpose();
		}
		Gd(l, 0) = (R + B_p.transpose() * K_p * B_p).inverse() * B_p.transpose() * temp * K_p * I_p;
		if (l == 0)
			Gd(l, 0) = (R + B_p.transpose() * K_p * B_p).inverse() * B_p.transpose() * K_p * I_p;
	}
	this->Gd = Gd;
	return Gd;
};

void Wonbin::Com::ShowGd()
{
	cout << Gd;
};

Wonbin::X_Com::X_Com() {
	sim_time = 8 * walktime;//수정
	sim_n = sim_time * freq;
	zmp_err_int = 0;
	u_prev = 0;
};
//수정
void Wonbin::X_Com::Change_Ref_Xpos(double a, double b, double c, double d, double e, double f, double g, double h, double i, double j, double k, double l)
{
	this->Ref_Xpos[0] = a;
	this->Ref_Xpos[1] = b;
	this->Ref_Xpos[2] = c;
	this->Ref_Xpos[3] = d;
	this->Ref_Xpos[4] = e;
	this->Ref_Xpos[5] = f;
    this->Ref_Xpos[6] = g;
    this->Ref_Xpos[7] = h;
    this->Ref_Xpos[8] = i;
    this->Ref_Xpos[9] = j;
    this->Ref_Xpos[10] = k;
    this->Ref_Xpos[11] = l;
};
//수정
MatrixXd Wonbin::X_Com::XComSimulation() {
	PreviewGd();
	RowVectorXd zmp_ref(sim_n);
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;
		if (time < 1.5 * walktime) {
			zmp_ref[i] = Ref_Xpos[0];
		}
		else if (time < 2 * walktime) {
			zmp_ref[i] = Ref_Xpos[1];
		}
		else if (time < 2.5 * walktime) {
			zmp_ref[i] = Ref_Xpos[2];
		}
		else if (time < 3 * walktime) {
			zmp_ref[i] = Ref_Xpos[3];
		}
		else if (time < 3.5 * walktime) {
			zmp_ref[i] = Ref_Xpos[4];
		}
		else if (time < 4 * walktime) {
			zmp_ref[i] = Ref_Xpos[5];
		}
        else if (time < 4.5 * walktime) {
			zmp_ref[i] = Ref_Xpos[6];
		}
        else if (time < 5 * walktime) {
			zmp_ref[i] = Ref_Xpos[7];
		}
        else if (time < 5.5 * walktime) {
			zmp_ref[i] = Ref_Xpos[8];
		}
        else if (time < 6 * walktime) {
			zmp_ref[i] = Ref_Xpos[9];
		}
        else if (time < 6.5 * walktime) {
			zmp_ref[i] = Ref_Xpos[10];
		}
        else if (time < 7 * walktime) {
			zmp_ref[i] = Ref_Xpos[11];
		}
		else
			zmp_ref[i] = Ref_Xpos[11];
	}
	RowVectorXd zmp_ref_fifo(NL);
	RowVectorXd u(sim_n);
	RowVectorXd zmp(sim_n);
	RowVectorXd zmp_ref_final(sim_n);
	RowVectorXd CP(sim_n);
	MatrixXd XCom(3, sim_n + 1);
	XCom = MatrixXd::Zero(3, sim_n + 1);
	double w = sqrt(g / z_c);
	for (int i = 0; i < sim_n; i++) {
		for (int j = 0; j < NL; j++) {
			if (i + j < sim_n) {
				zmp_ref_fifo[j] = zmp_ref[i + j];
			}
			else {
				zmp_ref_fifo[j] = zmp_ref[sim_n - 1];
			}
		}
		u_prev = 0;
		for (int j = 0; j < NL; j++) {
			u_prev += Gd(j, 0) * zmp_ref_fifo[j];
		}
		u[i] = Gi * zmp_err_int - Gx * XCom.col(i) + u_prev;

		XCom.col(i + 1) = A * XCom.col(i) + B * u[i];

		zmp[i] = C * XCom.col(i);

		zmp_err_int += (zmp_ref[i] - zmp[i]);

		CP[i] = XCom(0, i) + 1 / w * XCom(1, i);

		zmp_ref_final[i] = zmp_ref[i];

	}
	this->XCom = XCom;
	return XCom;
};
//수정
Wonbin::Y_Com::Y_Com()
{
	sim_time = 8 * walktime;
	sim_n = sim_time * freq;
	zmp_err_int = 0;
	u_prev = 0;
}
//수정
void Wonbin::Y_Com::Change_Ref_Ypos(double a, double b, double c, double d, double e, double f, double g, double h, double i, double j, double k, double l)
 {
	this->Ref_Ypos[0] = a;
	this->Ref_Ypos[1] = b;
	this->Ref_Ypos[2] = c;
	this->Ref_Ypos[3] = d;
	this->Ref_Ypos[4] = e;
	this->Ref_Ypos[5] = f;
    this->Ref_Ypos[6] = g;
    this->Ref_Ypos[7] = h;
    this->Ref_Ypos[8] = i;
    this->Ref_Ypos[9] = j;
    this->Ref_Ypos[10] = k;
    this->Ref_Ypos[11] = l;
}
//수정
MatrixXd Wonbin::Y_Com::YComSimulation() {
	PreviewGd();
	RowVectorXd zmp_ref(sim_n);
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;
		if (time < 1 * walktime) {
			zmp_ref[i] = 0;
		}
		else if (time < 1.5 * walktime) {
			zmp_ref[i] = Ref_Ypos[0];
		}
		else if (time < 2 * walktime) {
			zmp_ref[i] = Ref_Ypos[1];
		}
		else if (time < 2.5 * walktime) {
			zmp_ref[i] = Ref_Ypos[2];
		}
		else if (time < 3 * walktime) {
			zmp_ref[i] = Ref_Ypos[3];
		}
		else if (time < 3.5 * walktime) {
			zmp_ref[i] = Ref_Ypos[4];
		}
		else if (time < 4 * walktime) {
			zmp_ref[i] = Ref_Ypos[5];
		}
		else if (time < 4.5 * walktime) {
			zmp_ref[i] = Ref_Ypos[6];
		}
        else if (time < 5 * walktime) {
			zmp_ref[i] = Ref_Ypos[7];
		}
        else if (time < 5.5 * walktime) {
			zmp_ref[i] = Ref_Ypos[8];
		}
        else if (time < 6 * walktime) {
			zmp_ref[i] = Ref_Ypos[9];
		}
        else if (time < 6.5 * walktime) {
			zmp_ref[i] = Ref_Ypos[10];
		}
        else if (time < 7 * walktime) {
			zmp_ref[i] = Ref_Ypos[11];
		}
		else
			zmp_ref[i] = Ref_Ypos[11];
	}

	RowVectorXd zmp_ref_fifo(NL);
	RowVectorXd u(sim_n);
	RowVectorXd zmp(sim_n);
	RowVectorXd zmp_ref_final(sim_n);
	RowVectorXd CP(sim_n);
	MatrixXd YCom(3, sim_n + 1);
	YCom = MatrixXd::Zero(3, sim_n + 1);
	double w = sqrt(g / z_c);
	for (int i = 0; i < sim_n; i++) {
		for (int j = 0; j < NL; j++) {
			if (i + j < sim_n) {
				zmp_ref_fifo[j] = zmp_ref[i + j];
			}
			else {
				zmp_ref_fifo[j] = zmp_ref[sim_n - 1];
			}
		}
		u_prev = 0;
		for (int j = 0; j < NL; j++) {
			u_prev += Gd(j, 0) * zmp_ref_fifo[j];
		}
		u[i] = Gi * zmp_err_int - Gx * YCom.col(i) + u_prev;

		YCom.col(i + 1) = A * YCom.col(i) + B * u[i];

		zmp[i] = C * YCom.col(i);

		zmp_err_int += (zmp_ref[i] - zmp[i]);

		CP[i] = YCom(0, i) + 1 / w * YCom(1, i);

		zmp_ref_final[i] = zmp_ref[i];

	}
	this->YCom = YCom;
	return YCom;
}
//수정
Wonbin::Foot::Foot() {
	walkfreq = 1.48114;
	walktime = 1 / walkfreq;
	step = 0.05;
	freq = 500;
	XStep << 0, 0, 0, 0, 0, 0;
	XStride << 0, 0, 0, 0, 0, 0;
};

void Wonbin::Foot::Change_step(double a) {
	this->step = a;
}

MatrixXd Wonbin::Foot::Equation_solver(double t0, double t1, double start, double end)
{
	Matrix<double, 6, 6> A;
	Matrix<double, 6, 1> B;
	Matrix<double, 6, 1> X;
	A << 1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5),
		0, 1, 2 * t0, 3 * pow(t0, 2), 4 * pow(t0, 3), 5 * pow(t0, 4),
		0, 0, 2, 6 * t0, 12 * pow(t0, 2), 20 * pow(t0, 3),
		1, t1, pow(t1, 2), pow(t1, 3), pow(t1, 4), pow(t1, 5),
		0, 1, 2 * t1, 3 * pow(t1, 2), 4 * pow(t1, 3), 5 * pow(t1, 4),
		0, 0, 2, 6 * t1, 12 * pow(t1, 2), 20 * pow(t1, 3);
	B << start, 0, 0, end, 0, 0;
	X = A.colPivHouseholderQr().solve(B);
	return X;
};

double Wonbin::Foot::Step(double t)
{
	double X = XStep(0) + XStep(1) * t + XStep(2) * pow(t, 2) + XStep(3) * pow(t, 3) + XStep(4) * pow(t, 4) + XStep(5) * pow(t, 5);
	return X;
};

double Wonbin::Foot::Stride(double t)
{
	double X = XStride(0) + XStride(1) * t + XStride(2) * pow(t, 2) + XStride(3) * pow(t, 3) + XStride(4) * pow(t, 4) + XStride(5) * pow(t, 5);
	return X;
};
//수정
MatrixXd Wonbin::Foot::RF_xsimulation_straightwalk()
{
	this->XStep = Equation_solver(0, walktime * 0.3, 0, step);
	this->XStride = Equation_solver(0, walktime * 0.3, 0, 2 * step);
	int sim_n = 8 * walktime * freq;
	double del_t = 1 / freq;
	RowVectorXd Footpos1(sim_n);
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;
		if (time < 1.1 * walktime) {
			Footpos1[i] = 0;
		}
		else if (time < 1.4 * walktime) {
			Footpos1[i] = Step(time - 1.1 * walktime);
		}
		else if (time < 2.1 * walktime) {
			Footpos1[i] = step;
		}
		else if (time < 2.4 * walktime) {
			Footpos1[i] = Stride(time - 2.1 * walktime) + step;
		}
		else if (time < 3.1 * walktime) {
			Footpos1[i] = 3 * step;
		}
		else if (time < 3.4 * walktime) {
			Footpos1[i] = Stride(time - 3.1 * walktime) + 3 * step;
		}
        else if (time < 4.1 * walktime) {
			Footpos1[i] =  5 * step;
		}
        else if (time < 4.4 * walktime) {
			Footpos1[i] = Stride(time - 4.1 * walktime) + 5 * step;
		}
        else if (time < 5.1 * walktime) {
			Footpos1[i] = 7 * step;
		}
        else if (time < 5.4 * walktime) {
			Footpos1[i] = Stride(time - 5.1 * walktime) + 7 * step;
		}
        else if (time < 6.1 * walktime) {
			Footpos1[i] = 9 * step;
		}
        else if (time < 6.4 * walktime) {
			Footpos1[i] = Stride(time - 6.1 * walktime) + 9 * step;
		}
		else {
			Footpos1[i] = 11 * step;
		}

	};
	return Footpos1;
	
};
//수정
MatrixXd Wonbin::Foot::LF_xsimulation_straightwalk() {
	this->XStep = Equation_solver(0, walktime * 0.3, 0, step);
	this->XStride = Equation_solver(0, walktime * 0.3, 0, 2 * step);
	int sim_n = 8 * walktime * freq;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;

		if (time < 1.6 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 1.9 * walktime) {
			Footpos[i] = Stride(time - 1.6 * walktime);
		}
		else if (time < 2.6 * walktime) {
			Footpos[i] = 2 * step;
		}
		else if (time < 2.9 * walktime) {
			Footpos[i] = Stride(time - 2.6 * walktime) + 2 * step;
		}
		else if (time < 3.6 * walktime) {
			Footpos[i] = 4 * step;
		}
		else if (time < 3.9 * walktime) {
			Footpos[i] = Stride(time - 3.6 * walktime) + 4 * step;
		}
        else if (time < 4.6 * walktime) {
			Footpos[i] = 6 * step;
		}
        else if (time < 4.9 * walktime) {
			Footpos[i] = Stride(time - 4.6 * walktime) + 6 * step;
		}
        else if (time < 5.6 * walktime) {
			Footpos[i] = 8 * step;
		}
        else if (time < 5.9 * walktime) {
			Footpos[i] = Stride(time - 5.6 * walktime) + 8 * step;
		}
        else if (time < 6.6 * walktime) {
			Footpos[i] = 10 * step;
		}
        else if (time < 6.9 * walktime) {
			Footpos[i] = Step(time - 6.6 * walktime) + 10 * step;
		}
		else {
			Footpos[i] = 11 * step;
		}
	};
	return Footpos;


}
//수정
MatrixXd Wonbin::Foot::RF_zsimulation_straightwalk()
{
	this->XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	this->XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	int sim_n = 8 * walktime * freq;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;
		if (time < 1.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 1.25 * walktime) {
			Footpos[i] = Step(time - 1.05 * walktime);
		}
		else if (time < 1.45 * walktime) {
			Footpos[i] = Stride(time - 1.05 * walktime);
		}
		else if (time < 2.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 2.25 * walktime) {
			Footpos[i] = Step(time - 2.05 * walktime);
		}
		else if (time < 2.45 * walktime) {
			Footpos[i] = Stride(time - 2.05 * walktime);
		}
		else if (time < 3.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 3.25 * walktime) {
			Footpos[i] = Step(time - 3.05 * walktime);
		}
		else if (time < 3.45 * walktime) {
			Footpos[i] = Stride(time - 3.05 * walktime);
		}
        else if (time < 4.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 4.25 * walktime) {
			Footpos[i] = Step(time - 4.05 * walktime);
		}
		else if (time < 4.45 * walktime) {
			Footpos[i] = Stride(time - 4.05 * walktime);
		}
        else if (time < 5.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 5.25 * walktime) {
			Footpos[i] = Step(time - 5.05 * walktime);
		}
		else if (time < 5.45 * walktime) {
			Footpos[i] = Stride(time - 5.05 * walktime);
		}
        else if (time < 6.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 6.25 * walktime) {
			Footpos[i] = Step(time - 6.05 * walktime);
		}
        else if (time < 6.45 * walktime) {
			Footpos[i] = Stride(time - 6.05 * walktime);
		}
		else {
			Footpos[i] = 0;
		}
	};
	return Footpos;
};
//수정
MatrixXd Wonbin::Foot::LF_zsimulation_straightwalk() {
	this->XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	this->XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	int sim_n = 8 * walktime * freq;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;
		if (time < 1.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 1.75 * walktime) {
			Footpos[i] = Step(time - 1.55 * walktime);
		}
		else if (time < 1.95 * walktime) {
			Footpos[i] = Stride(time - 1.55 * walktime);
		}
		else if (time < 2.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 2.75 * walktime) {
			Footpos[i] = Step(time - 2.55 * walktime);
		}
		else if (time < 2.95 * walktime) {
			Footpos[i] = Stride(time - 2.55 * walktime);
		}
		else if (time < 3.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 3.75 * walktime) {
			Footpos[i] = Step(time - 3.55 * walktime);
		}
		else if (time < 3.95 * walktime) {
			Footpos[i] = Stride(time - 3.55 * walktime);
		}
        else if (time < 4.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 4.75 * walktime) {
			Footpos[i] = Step(time - 4.55 * walktime);
		}
		else if (time < 4.95 * walktime) {
			Footpos[i] = Stride(time - 4.55 * walktime);
		}
        else if (time < 5.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 5.75 * walktime) {
			Footpos[i] = Step(time - 5.55 * walktime);
		}
		else if (time < 5.95 * walktime) {
			Footpos[i] = Stride(time - 5.55 * walktime);
		}
        else if (time < 6.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 6.75 * walktime) {
			Footpos[i] = Step(time - 6.55 * walktime);
		}
		else if (time < 6.95 * walktime) {
			Footpos[i] = Stride(time - 6.55 * walktime);
		}
		else {
			Footpos[i] = 0;
		}
	};
	return Footpos;
};
//수정
MatrixXd Wonbin::Foot::RF_ysimulation_leftwalk() {
	this->XStep = Equation_solver(0, walktime * 0.3, 0, step);
	int sim_n = 8 * walktime * freq;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n); //rightfoot motion
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;
		if (time < 1.1 * walktime) {
			Footpos[i] = -step;
		}
		else if (time < 1.4 * walktime) {
			Footpos[i] = -step;
		}
		else if (time < 2.1 * walktime) {
			Footpos[i] = -step;
		}
		else if (time < 2.4 * walktime) {
			Footpos[i] = Step(time - 2.1 * walktime) - step;
		}
		else if (time < 3.1 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 3.4 * walktime) {
			Footpos[i] = Step(time - 3.1 * walktime);
		}
        else if (time < 4.1 * walktime) {
			Footpos[i] = step;
		}
        else if (time < 4.4 * walktime) {
			Footpos[i] = Step(time - 4.1 * walktime) + step;
		}
        else if (time < 5.1 * walktime) {
			Footpos[i] = 2*step;
		}
        else if (time < 5.4 * walktime) {
			Footpos[i] = Step(time - 5.1 * walktime) + 2*step;
		}
        else if (time < 6.1 * walktime) {
			Footpos[i] = 3*step;
		}
        else if (time < 6.4 * walktime) {
			Footpos[i] = Step(time - 6.1 * walktime) + 3*step;
		}
		else {
			Footpos[i] = 4*step;
		}
	};
	return Footpos;
}
//수정
MatrixXd Wonbin::Foot::LF_ysimulation_leftwalk() {
	this->XStep = Equation_solver(0, walktime * 0.3, 0, step);
	int sim_n = 8 * walktime * freq;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;

		if (time < 1.6 * walktime) {
			Footpos[i] = step;
		}
		else if (time < 1.9 * walktime) {
			Footpos[i] = Step(time - 1.6 * walktime) + step;
		}
		else if (time < 2.6 * walktime) {
			Footpos[i] = 2 * step;
		}
		else if (time < 2.9 * walktime) {
			Footpos[i] = Step(time - 2.6 * walktime) + 2 * step;
		}
		else if (time < 3.6 * walktime) {
			Footpos[i] = 3 * step;
		}
		else if (time < 3.9 * walktime) {
			Footpos[i] = Step(time - 3.6 * walktime) + 3 * step;
		}
        else if (time < 4.6 * walktime) {
			Footpos[i] = 4 * step;
		}
		else if (time < 4.9 * walktime) {
			Footpos[i] = Step(time - 4.6 * walktime) + 4 * step;
		}
        else if (time < 5.6 * walktime) {
			Footpos[i] = 5 * step;
		}
		else if (time < 5.9 * walktime) {
			Footpos[i] = Step(time - 5.6 * walktime) + 5 * step;
		}
         else if (time < 6.6 * walktime) {
			Footpos[i] = 6 * step;
		}
         else if (time < 6.9 * walktime) {
			Footpos[i] = 6 * step;
		}
		else {
			Footpos[i] = 6 * step;
		}
	};
	return Footpos;

}
//수정
MatrixXd Wonbin::Foot::RF_zsimulation_leftwalk() {
	this->XStep = Equation_solver(0, 0.2 * walktime, 0, 0.02);
	this->XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.02, 0);
	int sim_n = 8 * walktime * freq;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;

		if (time < 1.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 1.25 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 1.45 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 2.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 2.25 * walktime) {
			Footpos[i] = Step(time - 2.05 * walktime);
		}
		else if (time < 2.45 * walktime) {
			Footpos[i] = Stride(time - 2.05 * walktime);
		}
		else if (time < 3.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 3.25 * walktime) {
			Footpos[i] = Step(time - 3.05 * walktime);
		}
		else if (time < 3.45 * walktime) {
			Footpos[i] = Stride(time - 3.05 * walktime);
		}
        else if (time < 4.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 4.25 * walktime) {
			Footpos[i] = Step(time - 4.05 * walktime);
		}
		else if (time < 4.45 * walktime) {
			Footpos[i] = Stride(time - 4.05 * walktime);
		}
        else if (time < 5.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 5.25 * walktime) {
			Footpos[i] = Step(time - 5.05 * walktime);
		}
		else if (time < 5.45 * walktime) {
			Footpos[i] = Stride(time - 5.05 * walktime);
		}
        else if (time < 6.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 6.25 * walktime) {
			Footpos[i] = Step(time - 6.05 * walktime);
		}
		else if (time < 6.45 * walktime) {
			Footpos[i] = Stride(time - 6.05 * walktime);
		}
		else {
			Footpos[i] = 0;
		}
	};
	return Footpos;
};
//수정
MatrixXd Wonbin::Foot::LF_zsimulation_leftwalk() {
	this->XStep = Equation_solver(0, 0.2 * walktime, 0, 0.02);
	this->XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.02, 0);
	int sim_n = 8 * walktime * freq;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;

		if (time < 1.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 1.75 * walktime) {
			Footpos[i] = Step(time - 1.55 * walktime);
		}
		else if (time < 1.95 * walktime) {
			Footpos[i] = Stride(time - 1.55 * walktime);
		}
		else if (time < 2.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 2.75 * walktime) {
			Footpos[i] = Step(time - 2.55 * walktime);
		}
		else if (time < 2.95 * walktime) {
			Footpos[i] = Stride(time - 2.55 * walktime);
		}
		else if (time < 3.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 3.75 * walktime) {
			Footpos[i] = Step(time - 3.55 * walktime);
		}
		else if (time < 3.95 * walktime) {
			Footpos[i] = Stride(time - 3.55 * walktime);
		}
        else if (time < 4.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 4.75 * walktime) {
			Footpos[i] = Step(time - 4.55 * walktime);
		}
		else if (time < 4.95 * walktime) {
			Footpos[i] = Stride(time - 4.55 * walktime);
		}
        else if (time < 5.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 5.75 * walktime) {
			Footpos[i] = Step(time - 5.55 * walktime);
		}
		else if (time < 5.95 * walktime) {
			Footpos[i] = Stride(time - 5.55 * walktime);
		}
        else if (time < 6.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 6.75 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 6.95 * walktime) {
			Footpos[i] = 0;
		}
		else {
			Footpos[i] = 0;
		}
	};
	return Footpos;
};
//수정
MatrixXd Wonbin::Foot::RF_ysimulation_rightwalk() {

	this->XStep = Equation_solver(0, walktime * 0.3, 0, step);
	int sim_n = 8 * walktime * freq;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;

		if (time < 1.6 * walktime) {
			Footpos[i] = step;
		}
		else if (time < 1.9 * walktime) {
			Footpos[i] = Step(time - 1.6 * walktime) + step;
		}
		else if (time < 2.6 * walktime) {
			Footpos[i] = 2 * step;
		}
		else if (time < 2.9 * walktime) {
			Footpos[i] = Step(time - 2.6 * walktime) + 2 * step;
		}
		else if (time < 3.6 * walktime) {
			Footpos[i] = 3 * step;
		}
		else if (time < 3.9 * walktime) {
			Footpos[i] = Step(time - 2.6 * walktime) + 3 * step;
		}
        else if (time < 4.6 * walktime) {
			Footpos[i] = 4 * step;
		}
        else if (time < 4.9 * walktime) {
			Footpos[i] = Step(time - 2.6 * walktime) + 4 * step;
		}
        else if (time < 5.6 * walktime) {
			Footpos[i] = 5 * step;
		}
        else if (time < 5.9 * walktime) {
			Footpos[i] = Step(time - 2.6 * walktime) + 5 * step;
		}
        else if (time < 6.6 * walktime) {
			Footpos[i] = 6 * step;
		}
        else if (time < 6.9 * walktime) {
			Footpos[i] = 6 * step;
		}
		else {
			Footpos[i] = 6 * step;
		}
	};
	return Footpos;

}
//수정
MatrixXd Wonbin::Foot::LF_ysimulation_rightwalk() {
	this->XStep = Equation_solver(0, walktime * 0.3, 0, step);
	int sim_n = 8 * walktime * freq;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n); //rightfoot motion
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;
		if (time < 1.1 * walktime) {
			Footpos[i] = -step;
		}
		else if (time < 1.4 * walktime) {
			Footpos[i] = -step;
		}
		else if (time < 2.1 * walktime) {
			Footpos[i] = -step;
		}
		else if (time < 2.4 * walktime) {
			Footpos[i] = Step(time - 2.1 * walktime) - step;
		}
		else if (time < 3.1 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 3.4 * walktime) {
			Footpos[i] = Step(time - 3.1 * walktime);
		}
        else if (time < 4.1 * walktime) {
			Footpos[i] = step;
		}
		else if (time < 4.4 * walktime) {
			Footpos[i] = Step(time - 3.1 * walktime) + step;
		}
        else if (time < 5.1 * walktime) {
			Footpos[i] = 2 * step;
		}
		else if (time < 5.4 * walktime) {
			Footpos[i] = Step(time - 3.1 * walktime) + 2 * step;
		}
        else if (time < 6.1 * walktime) {
			Footpos[i] = 3 * step;
		}
		else if (time < 6.4 * walktime) {
			Footpos[i] = Step(time - 3.1 * walktime) + 3 * step;
		}
		else {
			Footpos[i] = 4 * step;
		}
	};
	return Footpos;
}
//수정
MatrixXd Wonbin::Foot::RF_zsimulation_rightwalk() {

	this->XStep = Equation_solver(0, 0.2 * walktime, 0, 0.02);
	this->XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.02, 0);
	int sim_n = 8 * walktime * freq;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;

		if (time < 1.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 1.75 * walktime) {
			Footpos[i] = Step(time - 1.55 * walktime);
		}
		else if (time < 1.95 * walktime) {
			Footpos[i] = Stride(time - 1.55 * walktime);
		}
		else if (time < 2.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 2.75 * walktime) {
			Footpos[i] = Step(time - 2.55 * walktime);
		}
		else if (time < 2.95 * walktime) {
			Footpos[i] = Stride(time - 2.55 * walktime);
		}
		else if (time < 3.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 3.75 * walktime) {
			Footpos[i] = Step(time - 3.55 * walktime);
		}
		else if (time < 3.95 * walktime) {
			Footpos[i] = Stride(time - 3.55 * walktime);
		}
        else if (time < 4.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 4.75 * walktime) {
			Footpos[i] = Step(time - 4.55 * walktime);
		}
		else if (time < 4.95 * walktime) {
			Footpos[i] = Stride(time - 4.55 * walktime);
		}
        else if (time < 5.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 5.75 * walktime) {
			Footpos[i] = Step(time - 5.55 * walktime);
		}
		else if (time < 5.95 * walktime) {
			Footpos[i] = Stride(time - 5.55 * walktime);
		}
        else if (time < 6.55 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 6.75 * walktime) {
			Footpos[i] = 0;
            }
		else if (time < 6.95 * walktime) {
			Footpos[i] = 0;
		}
		else {
			Footpos[i] = 0;
		}
	};
	return Footpos;
};
//수정
MatrixXd Wonbin::Foot::LF_zsimulation_rightwalk() {
	this->XStep = Equation_solver(0, 0.2 * walktime, 0, 0.02);
	this->XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.02, 0);
	int sim_n = 8 * walktime * freq;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++) {
		double time = i * del_t;

		if (time < 1.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 1.25 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 1.45 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 2.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 2.25 * walktime) {
			Footpos[i] = Step(time - 2.05 * walktime);
		}
		else if (time < 2.45 * walktime) {
			Footpos[i] = Stride(time - 2.05 * walktime);
		}
		else if (time < 3.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 3.25 * walktime) {
			Footpos[i] = Step(time - 3.05 * walktime);
		}
		else if (time < 3.45 * walktime) {
			Footpos[i] = Stride(time - 3.05 * walktime);
		}
        else if (time < 4.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 4.25 * walktime) {
			Footpos[i] = Step(time - 4.05 * walktime);
		}
		else if (time < 4.45 * walktime) {
			Footpos[i] = Stride(time - 4.05 * walktime);
		}
        else if (time < 5.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 5.25 * walktime) {
			Footpos[i] = Step(time - 5.05 * walktime);
		}
		else if (time < 5.45 * walktime) {
			Footpos[i] = Stride(time - 5.05 * walktime);
		}
        else if (time < 6.05 * walktime) {
			Footpos[i] = 0;
		}
		else if (time < 6.25 * walktime) {
			Footpos[i] = Step(time - 6.05 * walktime);
		}
		else if (time < 6.45 * walktime) {
			Footpos[i] = Stride(time - 6.05 * walktime);
		}
		else {
			Footpos[i] = 0;
		}
	};
	return Footpos;
};
//수정
Wonbin::BRP_Inverse_Kinematics::BRP_Inverse_Kinematics() {

	walkfreq = 1.48114;
	walktime = 1 / walkfreq;
	stride = 0.1;
	freq = 500;
	del_t = 1 / freq;
	sim_time = 8 * walktime;
	sim_n = sim_time * freq;


	RL_th[0] = 0. * deg2rad;      // RHY
	RL_th[1] = 0. * deg2rad;		// RHR
	RL_th[2] = -35. * deg2rad;	// RHP
	RL_th[3] = 70. * deg2rad;		// RKN
	RL_th[4] = -35. * deg2rad;    // RAP
	RL_th[5] = 0. * deg2rad;		// RAR

	LL_th[0] = 0. * deg2rad;		// LHY
	LL_th[1] = 0. * deg2rad;		// LHR
	LL_th[2] = -35. * deg2rad;	// LHP
	LL_th[3] = 70. * deg2rad;		// LKN
	LL_th[4] = -35. * deg2rad;	// LAP
	LL_th[5] = 0. * deg2rad;		// LAR

	Ref_RL_PR[0] = 40.;
	Ref_RL_PR[1] = -L0;
	Ref_RL_PR[2] = -L1 - L2 - L3 - L4 - L5 - L6 + 40.;
	Ref_RL_PR[3] = 0 * deg2rad;
	Ref_RL_PR[4] = 0 * deg2rad;
	Ref_RL_PR[5] = 0 * deg2rad;

	Ref_LL_PR[0] = 40.;
	Ref_LL_PR[1] = L0;
	Ref_LL_PR[2] = -L1 - L2 - L3 - L4 - L5 - L6 + 40.;
	Ref_LL_PR[3] = 0 * deg2rad;
	Ref_LL_PR[4] = 0 * deg2rad;
	Ref_LL_PR[5] = 0 * deg2rad;
};

void Wonbin::BRP_Inverse_Kinematics::BRP_RL_FK(double th[6], double PR[6]) {
	double  c1, c2, c3, c4, c5, c6, s1, s2, s3, s4, s5, s6, nx, ny, nz, ox, oy, oz, ax, ay, az;

	c1 = cos(th[0]); c2 = cos(th[1]); c3 = cos(th[2]); c4 = cos(th[3]); c5 = cos(th[4]); c6 = cos(th[5]);
	s1 = sin(th[0]); s2 = sin(th[1]); s3 = sin(th[2]); s4 = sin(th[3]); s5 = sin(th[4]); s6 = sin(th[5]);


	// Endeffector position 
	PR[0] = L5 * s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3)) - L4 * s4 * (c1 * c3 - s1 * s2 * s3) - L5 * c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - L3 * c1 * s3 - L4 * c4 * (c1 * s3 + c3 * s1 * s2) - L2 * s1 * s2 - L6 * c6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) - L3 * c3 * s1 * s2 - L6 * c2 * s1 * s6;
	PR[1] = L2 * c1 * s2 - L4 * c4 * (s1 * s3 - c1 * c3 * s2) - L4 * s4 * (c3 * s1 + c1 * s2 * s3) - L5 * c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - L0 + L5 * s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3)) - L3 * s1 * s3 - L6 * c6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) + L3 * c1 * c3 * s2 + L6 * c1 * c2 * s6;
	PR[2] = L6 * s2 * s6 - L2 * c2 - L6 * c6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3)) - L3 * c2 * c3 - L1 - L5 * c5 * (c2 * c3 * c4 - c2 * s3 * s4) + L5 * s5 * (c2 * c3 * s4 + c2 * c4 * s3) - L4 * c2 * c3 * c4 + L4 * c2 * s3 * s4;

	// Endeffector orientation
	nx = -c5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3)) - s5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2));
	ny = -c5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3)) - s5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2));
	nz = -c5 * (c2 * c3 * s4 + c2 * c4 * s3) - s5 * (c2 * c3 * c4 - c2 * s3 * s4);

	//printf("nx = %f, ny = %f, nz = %f \n",nx,ny,nz);

	ox = s6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) - c2 * c6 * s1;
	oy = s6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) + c1 * c2 * c6;
	oz = c6 * s2 + s6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3));

	//printf("ox = %f, oy = %f, oz = %f \n",ox,oy,oz);

	ax = c6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) + c2 * s1 * s6;
	ay = c6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) - c1 * c2 * s6;
	az = c6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3)) - s2 * s6;

	//printf("ax = %f, ay = %f, az = %f \n",ax,ay,az);

	PR[5] = atan2(ny, nx);  // FI : Roll about z0 axis
	PR[4] = atan2(-nz, cos(PR[5]) * nx + sin(PR[5]) * ny);  // theta : Pitch about y0 axis
	PR[3] = atan2(sin(PR[5]) * ax - cos(PR[5]) * ay, -sin(PR[5]) * ox + cos(PR[5]) * oy);
};

void Wonbin::BRP_Inverse_Kinematics::BRP_RL_IK(double Ref_RL_RP[6], double Init_th[6], double IK_th[6]) {
	int iter, i, j, k;
	double th[6] = { 0.,0.,0.,0.,0.,0. }, PR[6] = { 0.,0.,0.,0.,0.,0. }, old_PR[6] = { 0.,0.,0.,0.,0.,0. }, F[6] = { 0.,0.,0.,0.,0.,0. }, old_Q[6] = { 0.,0.,0.,0.,0.,0. },
		New_Q4J[6] = { 0.,0.,0.,0.,0.,0. }, ERR = 0., sum = 0.,
		New_PR[6][6] = { {0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.} },
		J[6][6] = { {0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.} },
		Inv_J[6][6] = { {0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.} };
	const double del_Q = 0.0001;
	for (i = 0; i < 6; i++) th[i] = Init_th[i];
	for (iter = 0; iter < 100; iter++) {

		for (i = 0; i < 6; i++)	old_Q[i] = th[i];
		BRP_RL_FK(th, PR);

		for (i = 0; i < 6; i++) F[i] = Ref_RL_RP[i] - PR[i];


		ERR = sqrt(F[0] * F[0] + F[1] * F[1] + F[2] * F[2] + F[3] * F[3] + F[4] * F[4] + F[5] * F[5]);

		if (ERR < 0.0001) {
			for (i = 0; i < 6; i++)	IK_th[i] = th[i];
			break;
		}
		else if (iter == 99) {
			for (i = 0; i < 6; i++)	IK_th[i] = Init_th[i];

			break;
		}

		// Jacobian Cacluation using perturbation //
		for (i = 0; i < 6; i++) old_PR[i] = PR[i];

		for (i = 0; i < 6; i++) {

			for (j = 0; j < 6; j++) New_Q4J[j] = old_Q[j];  // Reset

			New_Q4J[i] = old_Q[i] + del_Q; // Perturb

			// Forward Kinematics again //
			BRP_RL_FK(New_Q4J, PR);

			for (j = 0; j < 6; j++) New_PR[j][i] = PR[j];
		} // End of for(i=0; i<6; i++){

		for (i = 0; i < 6; i++)
			for (j = 0; j < 6; j++)
				J[i][j] = (New_PR[i][j] - old_PR[i]) / del_Q;


		//inversMatrix(J, Inv_J);
		inv_mat6(0, 6, 0, J, 0, Inv_J);


		for (k = 0; k < 6; k++)
		{
			sum = 0.;
			for (j = 0; j < 6; j++)
			{
				sum = sum + Inv_J[k][j] * F[j];
			}
			th[k] = old_Q[k] + sum;
		}
		//printf("R_iter = %d, ERR = %f,	th[0] = %10.8f,	th[1] = %10.8f,	th[2] = %10.8f \n",iter, ERR, th[0], th[1], th[2]);

		if (th[3] < 0) {  // ���������� (-)  �� �����ϴ� ���� ���� 
			th[3] = -th[3];
		}


	}

}

void Wonbin::BRP_Inverse_Kinematics::BRP_LL_FK(double th[6], double PR[6]) {
	double  c1, c2, c3, c4, c5, c6, s1, s2, s3, s4, s5, s6, nx, ny, nz, ox, oy, oz, ax, ay, az;

	c1 = cos(th[0]); c2 = cos(th[1]); c3 = cos(th[2]); c4 = cos(th[3]); c5 = cos(th[4]); c6 = cos(th[5]);
	s1 = sin(th[0]); s2 = sin(th[1]); s3 = sin(th[2]); s4 = sin(th[3]); s5 = sin(th[4]); s6 = sin(th[5]);


	// Endeffector position
	PR[0] = L5 * s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3)) - L4 * s4 * (c1 * c3 - s1 * s2 * s3) - L5 * c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - L3 * c1 * s3 - L4 * c4 * (c1 * s3 + c3 * s1 * s2) - L2 * s1 * s2 - L6 * c6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) - L3 * c3 * s1 * s2 - L6 * c2 * s1 * s6;
	PR[1] = L0 - L4 * c4 * (s1 * s3 - c1 * c3 * s2) - L4 * s4 * (c3 * s1 + c1 * s2 * s3) - L5 * c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) + L2 * c1 * s2 + L5 * s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3)) - L3 * s1 * s3 - L6 * c6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) + L3 * c1 * c3 * s2 + L6 * c1 * c2 * s6;
	PR[2] = L6 * s2 * s6 - L2 * c2 - L6 * c6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3)) - L3 * c2 * c3 - L1 - L5 * c5 * (c2 * c3 * c4 - c2 * s3 * s4) + L5 * s5 * (c2 * c3 * s4 + c2 * c4 * s3) - L4 * c2 * c3 * c4 + L4 * c2 * s3 * s4;


	// Endeffector orientation
	nx = -c5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3)) - s5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2));
	ny = -c5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3)) - s5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2));
	nz = -c5 * (c2 * c3 * s4 + c2 * c4 * s3) - s5 * (c2 * c3 * c4 - c2 * s3 * s4);

	ox = s6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) - c2 * c6 * s1;
	oy = s6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) + c1 * c2 * c6;
	oz = c6 * s2 + s6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3));

	ax = c6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) + c2 * s1 * s6;
	ay = c6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) - c1 * c2 * s6;
	az = c6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3)) - s2 * s6;

	PR[5] = atan2(ny, nx);  // FI : Roll about z0 axis
	PR[4] = atan2(-nz, cos(PR[5]) * nx + sin(PR[5]) * ny);  // theta : Pitch about y0 axis
	PR[3] = atan2(sin(PR[5]) * ax - cos(PR[5]) * ay, -sin(PR[5]) * ox + cos(PR[5]) * oy);  // csi : Yaw about x0 axis

};

void Wonbin::BRP_Inverse_Kinematics::BRP_LL_IK(double Ref_LL_RP[6], double Init_th[6], double IK_th[6]) {
	int iter, i, j, k;
	double th[6] = { 0.,0.,0.,0.,0.,0. }, PR[6] = { 0.,0.,0.,0.,0.,0. }, old_PR[6] = { 0.,0.,0.,0.,0.,0. }, F[6] = { 0.,0.,0.,0.,0.,0. }, old_Q[6] = { 0.,0.,0.,0.,0.,0. },
		New_Q4J[6] = { 0.,0.,0.,0.,0.,0. }, ERR = 0., sum = 0.,
		New_PR[6][6] = { {0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.} },
		J[6][6] = { {0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.} },
		Inv_J[6][6] = { {0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.} };
	const double del_Q = 0.0001;


	// Initial Joint angles //
	for (i = 0; i < 6; i++) th[i] = Init_th[i];


	// printf("th[0] = %10.8f,	th[1] = %10.8f,	th[2] = %10.8f \n",th[0], th[1], th[2]);


	for (iter = 0; iter < 100; iter++) {

		//printf("iter = %d, ERR = %f \n",iter, ERR);

		for (i = 0; i < 6; i++)	old_Q[i] = th[i];

		// Forward Kinematics
		BRP_LL_FK(th, PR);

		// Error calculation
		for (i = 0; i < 6; i++) F[i] = Ref_LL_RP[i] - PR[i];


		ERR = sqrt(F[0] * F[0] + F[1] * F[1] + F[2] * F[2] + F[3] * F[3] + F[4] * F[4] + F[5] * F[5]);

		if (ERR < 0.0001) {
			for (i = 0; i < 6; i++)	IK_th[i] = th[i];

			break;
		}
		else if (iter == 99) {
			for (i = 0; i < 6; i++)	IK_th[i] = Init_th[i];

			break;
		}

		// Jacobian Cacluation using perturbation //
		for (i = 0; i < 6; i++) old_PR[i] = PR[i];

		for (i = 0; i < 6; i++) {

			for (j = 0; j < 6; j++) New_Q4J[j] = old_Q[j];  // Reset

			New_Q4J[i] = old_Q[i] + del_Q; // Perturb

			// Forward Kinematics again //
			BRP_LL_FK(New_Q4J, PR);

			for (j = 0; j < 6; j++) New_PR[j][i] = PR[j];
		} // End of for(i=0; i<6; i++){

		for (i = 0; i < 6; i++)
			for (j = 0; j < 6; j++)
				J[i][j] = (New_PR[i][j] - old_PR[i]) / del_Q;


		inv_mat6(0, 6, 0, J, 0, Inv_J);


		//printf("old_Q[0] = %10.8f,	old_Q[1] = %10.8f,	old_Q[2] = %10.8f \n",old_Q[0], old_Q[1], old_Q[2]);

		for (k = 0; k < 6; k++)
		{
			sum = 0.;
			for (j = 0; j < 6; j++)
			{
				sum = sum + Inv_J[k][j] * F[j];
			}
			//printf("sum = %10.8f\n",sum);

			th[k] = old_Q[k] + sum;
		}

		//printf("R_iter = %d, ERR = %f,	th[0] = %10.8f,	th[1] = %10.8f,	th[2] = %10.8f \n",iter, ERR, th[0], th[1], th[2]);
		if (th[3] < 0) {  // ���������� (-)  �� �����ϴ� ���� ���� 
			th[3] = -th[3];
		}


	} // End of for (iter = 0; iter <= 100; iter++) {

};

void Wonbin::BRP_Inverse_Kinematics::inv_mat6(int m, int n, double Mat4[][4], double Mat6[][6], double c_inv4[4][4], double c_inv[6][6]) {
	double big, size, abig, cbig, ratio;
	int i, k, j, ibig;
	double jcob[6][6];

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			if (n == 4)
				jcob[i][j] = Mat4[i][j];
			else if (n == 6)
				jcob[i][j] = Mat6[i][j];
		}
	}

	if (n == 6) {
		for (i = m; i < n; i++)
		{
			for (j = m; j < n; j++)
			{
				c_inv[i][j] = 0.;
				if (i == j) c_inv[i][i] = 1.;
			}
		}
		for (k = m; k < n; k++)
		{
			big = fabs(jcob[k][k]);
			ibig = k;

			for (i = k; i < n; i++)
			{
				size = fabs(jcob[i][k]);
				if (size < big) goto next;
				big = size;
				ibig = i;
			next:;
			}

			if (k == ibig) goto next2;
			for (j = m; j < n; j++)
			{
				if (j >= k)
				{
					abig = jcob[ibig][j];
					jcob[ibig][j] = jcob[k][j];
					jcob[k][j] = abig;
				}
				cbig = c_inv[ibig][j];
				c_inv[ibig][j] = c_inv[k][j];
				c_inv[k][j] = cbig;
			}




		next2:;

			if (jcob[k][k] == 0.) {/*lcd_putch('S'); halt_e();return(1);*/ }
			for (i = m; i < n; i++)
			{
				if (i == k) goto next3;
				ratio = jcob[i][k] / jcob[k][k];
				for (j = m; j < n; j++)
				{
					if (j >= k) jcob[i][j] = jcob[i][j] - ratio * jcob[k][j];
					c_inv[i][j] = c_inv[i][j] - ratio * c_inv[k][j];
				}


			next3:;

			}



		}
		for (k = m; k < n; k++)
		{
			for (j = m; j < n; j++)
			{
				c_inv[k][j] = c_inv[k][j] / jcob[k][k];
			}
		}
	}




	else if (n == 4) {
		for (i = m; i < n; i++)
		{
			for (j = m; j < n; j++)
			{
				c_inv4[i][j] = 0.;
				if (i == j) c_inv4[i][i] = 1.;
			}
		}
		for (k = m; k < n; k++)
		{
			big = fabs(jcob[k][k]);
			ibig = k;

			for (i = k; i < n; i++)
			{
				size = fabs(jcob[i][k]);
				if (size < big) goto next1_1;
				big = size;
				ibig = i;
			next1_1:;
			}

			if (k == ibig) goto next2_1;
			for (j = m; j < n; j++)
			{
				if (j >= k)
				{
					abig = jcob[ibig][j];
					jcob[ibig][j] = jcob[k][j];
					jcob[k][j] = abig;
				}
				cbig = c_inv4[ibig][j];
				c_inv4[ibig][j] = c_inv4[k][j];
				c_inv4[k][j] = cbig;
			}




		next2_1:;

			if (jcob[k][k] == 0.) {/*lcd_putch('S'); halt_e();return(1);*/ }
			for (i = m; i < n; i++)
			{
				if (i == k) goto next3_1;
				ratio = jcob[i][k] / jcob[k][k];
				for (j = m; j < n; j++)
				{
					if (j >= k) jcob[i][j] = jcob[i][j] - ratio * jcob[k][j];
					c_inv4[i][j] = c_inv4[i][j] - ratio * c_inv4[k][j];
				}


			next3_1:;

			}

		}
		for (k = m; k < n; k++)
		{
			for (j = m; j < n; j++)
			{
				c_inv4[k][j] = c_inv4[k][j] / jcob[k][k];
			}
		}
	}

}

MatrixXd Wonbin::BRP_Inverse_Kinematics::BRP_RL_Simulation(MatrixXd RFx, MatrixXd RFy, MatrixXd RFz) {

	unsigned long Index_CNT = 0;
	BRP_RL_IK(Ref_RL_PR, RL_th, RL_th_IK);
	MatrixXd RL = MatrixXd::Zero(sim_n,6);
	for (Index_CNT = 0; Index_CNT < sim_n; Index_CNT++) {
		int i = 0;
		Ref_RL_PR[0] = 40 + 1000 * RFx(0, Index_CNT);
		Ref_RL_PR[1] = 1000 * RFy(0, Index_CNT);
		Ref_RL_PR[2] = -L1 - L2 - L3 - L4 - L5 - L6 + 40. + 1000 * RFz(0, Index_CNT);
		Ref_RL_PR[3] = 0 * deg2rad;
		Ref_RL_PR[4] = 0 * deg2rad;
		Ref_RL_PR[5] = 0 * deg2rad;
		if (Index_CNT == 0) {

			RL_th[0] = 0. * deg2rad;
			RL_th[1] = 0. * deg2rad;
			RL_th[2] = -15. * deg2rad; 	//  
			RL_th[3] = 30. * deg2rad;		// RKN
			RL_th[4] = -15. * deg2rad;    // RAP
			RL_th[5] = 0. * deg2rad;		// RAR
		}
		BRP_RL_IK(Ref_RL_PR, RL_th, RL_th_IK);
		for (i = 0; i < 6; i++) {

			RL_th[i] = RL_th_IK[i];
		}
		RL(Index_CNT, 0) = RL_th[0];
		RL(Index_CNT, 1) = RL_th[1];
		RL(Index_CNT, 2) = RL_th[2];
		RL(Index_CNT, 3) = RL_th[3];
		RL(Index_CNT, 4) = RL_th[4];
		RL(Index_CNT, 5) = RL_th[5];
	}
	return RL;
};

MatrixXd Wonbin::BRP_Inverse_Kinematics::BRP_LL_Simulation(MatrixXd LFx, MatrixXd LFy, MatrixXd LFz) {
	unsigned long Index_CNT = 0;
	BRP_LL_IK(Ref_LL_PR, LL_th, LL_th_IK);
	MatrixXd LL = MatrixXd::Zero(sim_n,6);

	for (Index_CNT = 0; Index_CNT < sim_n; Index_CNT++) {  
		int i = 0;

		Ref_LL_PR[0] = 40 + 1000 * LFx(0, Index_CNT);
		Ref_LL_PR[1] = 1000 * LFy(0, Index_CNT);
		Ref_LL_PR[2] = -L1 - L2 - L3 - L4 - L5 - L6 + 40. + 1000 * LFz(0, Index_CNT);
		Ref_LL_PR[3] = 0 * deg2rad;
		Ref_LL_PR[4] = 0 * deg2rad;
		Ref_LL_PR[5] = 0 * deg2rad;

		if (Index_CNT == 0) {

			LL_th[0] = 0. * deg2rad;		// LHY
			LL_th[1] = 0. * deg2rad;		// LHR
			LL_th[2] = -15. * deg2rad;	// LHP
			LL_th[3] = 30. * deg2rad;		// LKN
			LL_th[4] = -15. * deg2rad;	// LAP
			LL_th[5] = 0. * deg2rad;		// LAR

		};
		BRP_LL_IK(Ref_LL_PR, LL_th, LL_th_IK);
		for (i = 0; i < 6; i++) {

			LL_th[i] = LL_th_IK[i];

		}
		LL(Index_CNT, 0) = LL_th[0];
		LL(Index_CNT, 1) = LL_th[1];
		LL(Index_CNT, 2) = LL_th[2];
		LL(Index_CNT, 3) = LL_th[3];
		LL(Index_CNT, 4) = LL_th[4];
		LL(Index_CNT, 5) = LL_th[5];
	}
	return LL;
}
//수정
//Motions constructor
Wonbin::Motions::Motions() {
	walkfreq = 1.48114;
	walktime = 1 / walkfreq;
	stride = 0.1;
	freq = 500;
	del_t = 1 / freq;
	sim_time = 8 * walktime;
	sim_n = sim_time * freq;

	Motion0_RL = MatrixXd::Zero(6, sim_n);
	Motion0_LL = MatrixXd::Zero(6, sim_n);
	//Motion1_RL = MatrixXd::Zero(6, sim_n);
	//Motion1_LL = MatrixXd::Zero(6, sim_n);
};

//Moton1 go straight 4step
void Wonbin::Motions::Motion0() {
	MatrixXd relativeRFx = MatrixXd::Zero(1, sim_n);
	MatrixXd relativeLFx = MatrixXd::Zero(1, sim_n);
	MatrixXd RF_yFoot = -L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd LF_yFoot = L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd relativeRFz = MatrixXd::Zero(1, sim_n);
	MatrixXd relativeLFz = MatrixXd::Zero(1, sim_n);
	BRP_Inverse_Kinematics joint;
	this->Motion0_RL = joint.BRP_RL_Simulation(relativeRFx, RF_yFoot, relativeRFz);
	this->Motion0_LL = joint.BRP_LL_Simulation(relativeLFx, LF_yFoot, relativeLFz);
}
MatrixXd Wonbin::Motions::Return_Motion0_RL() {
	return Motion0_RL;
};
MatrixXd Wonbin::Motions::Return_Motion0_LL() {
	return Motion0_LL;
};
//수정
void Wonbin::Motions::Motion1() {

	X_Com XCOM;
	Y_Com YCOM;
	Foot Foot;
	XCOM.Change_Ref_Xpos(0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1);
	MatrixXd Xcom = XCOM.XComSimulation();
	YCOM.Change_Ref_Ypos(0.045, -0.045, 0.045, -0.045, 0.045, -0.045, 0.045, -0.045, 0.045, -0.045, 0.045, 0);
	MatrixXd Ycom = YCOM.YComSimulation();
	Foot.Change_step(0.1);
	MatrixXd LF_xFoot = Foot.LF_xsimulation_straightwalk();
	MatrixXd RF_xFoot = Foot.RF_xsimulation_straightwalk();
	MatrixXd RF_yFoot = -L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd LF_yFoot = L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd RF_zFoot = Foot.RF_zsimulation_straightwalk();
	MatrixXd LF_zFoot = Foot.LF_zsimulation_straightwalk();
	MatrixXd relativeRFx = RF_xFoot.block(0, 0, RF_xFoot.rows(), sim_n) - Xcom.block(0, 0, RF_xFoot.rows(), sim_n);
	MatrixXd relativeLFx = LF_xFoot.block(0, 0, LF_xFoot.rows(), sim_n) - Xcom.block(0, 0, LF_xFoot.rows(), sim_n);
	MatrixXd relativeRFy = RF_yFoot.block(0, 0, RF_yFoot.rows(), sim_n) - Ycom.block(0, 0, RF_yFoot.rows(), sim_n);
	MatrixXd relativeLFy = LF_yFoot.block(0, 0, LF_yFoot.rows(), sim_n) - Ycom.block(0, 0, LF_yFoot.rows(), sim_n);
	BRP_Inverse_Kinematics joint;
	this->Motion1_RL = joint.BRP_RL_Simulation(relativeRFx, relativeRFy, RF_zFoot);
	this->Motion1_LL = joint.BRP_LL_Simulation(relativeLFx, relativeLFy, LF_zFoot);
	cout << Xcom;
}
MatrixXd Wonbin::Motions::Return_Motion1_RL() {
	return Motion1_RL;
};
MatrixXd Wonbin::Motions::Return_Motion1_LL() {
	return Motion1_LL;
};
//수정
void Wonbin::Motions::Motion2() {
	Y_Com YCOM;
	Foot Foot;
	YCOM.Change_Ref_Ypos(0, -L0, 2 * L0, 0, 3 * L0, L0, 4 * L0, 2 * L0, 5 * L0, 3 * L0, 6 * L0, 5 * L0);
	MatrixXd Ycom = YCOM.YComSimulation();
	Foot.Change_step(L0);
	MatrixXd RF_yFoot = Foot.RF_ysimulation_leftwalk();
	MatrixXd LF_yFoot = Foot.LF_ysimulation_leftwalk();
	MatrixXd RF_zFoot = Foot.RF_zsimulation_leftwalk();
	MatrixXd LF_zFoot = Foot.LF_zsimulation_leftwalk();
	MatrixXd relativeRFx = MatrixXd::Zero(1, sim_n);
	MatrixXd relativeLFx = MatrixXd::Zero(1, sim_n);
	MatrixXd relativeRFy = RF_yFoot.block(0, 0, RF_yFoot.rows(), sim_n) - Ycom.block(0, 0, RF_yFoot.rows(), sim_n);
	MatrixXd relativeLFy = LF_yFoot.block(0, 0, LF_yFoot.rows(), sim_n) - Ycom.block(0, 0, LF_yFoot.rows(), sim_n);
	BRP_Inverse_Kinematics joint;
	this->Motion2_RL = joint.BRP_RL_Simulation(relativeRFx, relativeRFy, RF_zFoot);
	this->Motion2_LL = joint.BRP_LL_Simulation(relativeLFx, relativeLFy, LF_zFoot);

}
MatrixXd Wonbin::Motions::Return_Motion2_RL() {
	return Motion2_RL;
};
MatrixXd Wonbin::Motions::Return_Motion2_LL() {
	return Motion2_LL;
};
//수정
void Wonbin::Motions::Motion3() {
	Foot Foot;
	Y_Com YCOM;
	YCOM.Change_Ref_Ypos(0.045, -0.045, 0.045, -0.045, 0.045, -0.045, 0.045, -0.045, 0.045, -0.045, 0.045, 0);
	MatrixXd Ycom = YCOM.YComSimulation();
	MatrixXd RF_yFoot = -L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd LF_yFoot = L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd RF_zFoot = Foot.RF_zsimulation_straightwalk();
	MatrixXd LF_zFoot = Foot.LF_zsimulation_straightwalk();
	MatrixXd relativeRFx = MatrixXd::Zero(1, sim_n);
	MatrixXd relativeLFx = MatrixXd::Zero(1, sim_n);
	MatrixXd relativeRFy = RF_yFoot - Ycom.block(0, 0, RF_yFoot.rows(), sim_n);
	MatrixXd relativeLFy = LF_yFoot - Ycom.block(0, 0, LF_yFoot.rows(), sim_n);
	BRP_Inverse_Kinematics joint;
	this->Motion3_RL = joint.BRP_RL_Simulation(relativeRFx, relativeRFy, RF_zFoot);
	this->Motion3_LL = joint.BRP_LL_Simulation(relativeLFx, relativeLFy, LF_zFoot);
}
MatrixXd Wonbin::Motions::Return_Motion3_RL() {
	return Motion3_RL;
};
MatrixXd Wonbin::Motions::Return_Motion3_LL() {
	return Motion3_LL;
};
//수정
void Wonbin::Motions::Motion4() {

	Y_Com YCOM;
	Foot Foot;
	YCOM.Change_Ref_Ypos(0, -L0, 2 * L0, 0, 3 * L0, L0, 4 * L0, 2 * L0, 5 * L0, 3 * L0, 6 * L0, 5 * L0);
	MatrixXd Ycom = YCOM.YComSimulation();
	Foot.Change_step(-L0);
	MatrixXd RF_yFoot = Foot.RF_ysimulation_rightwalk();
	MatrixXd LF_yFoot = Foot.LF_ysimulation_rightwalk();
	MatrixXd RF_zFoot = Foot.RF_zsimulation_rightwalk();
	MatrixXd LF_zFoot = Foot.LF_zsimulation_rightwalk();
	MatrixXd relativeRFx = MatrixXd::Zero(1, sim_n);
	MatrixXd relativeLFx = MatrixXd::Zero(1, sim_n);
	MatrixXd relativeRFy = RF_yFoot.block(0, 0, RF_yFoot.rows(), sim_n) - Ycom.block(0, 0, RF_yFoot.rows(), sim_n);
	MatrixXd relativeLFy = LF_yFoot.block(0, 0, LF_yFoot.rows(), sim_n) - Ycom.block(0, 0, LF_yFoot.rows(), sim_n);
	BRP_Inverse_Kinematics joint;
	this->Motion4_RL = joint.BRP_RL_Simulation(relativeRFx, relativeRFy, RF_zFoot);
	this->Motion4_LL = joint.BRP_LL_Simulation(relativeLFx, relativeLFy, LF_zFoot);

}
MatrixXd Wonbin::Motions::Return_Motion4_RL() {
	return Motion4_RL;
};
MatrixXd Wonbin::Motions::Return_Motion4_LL() {
	return Motion4_LL;
};
//수정
void Wonbin::Motions::Motion5() {
	X_Com XCOM;
	Y_Com YCOM;
	Foot Foot;
	XCOM.Change_Ref_Xpos(0, -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7, -0.8, -0.9, -1.0, -1.1);
	MatrixXd Xcom = XCOM.XComSimulation();
	YCOM.Change_Ref_Ypos(0.045, -0.045, 0.045, -0.045, 0.045, -0.045, 0.045, -0.045, 0.045, -0.045, 0.045, 0);
	MatrixXd Ycom = YCOM.YComSimulation();
	Foot.Change_step(-0.1);
	MatrixXd LF_xFoot = Foot.LF_xsimulation_straightwalk();
	MatrixXd RF_xFoot = Foot.RF_xsimulation_straightwalk();
	MatrixXd RF_yFoot = -L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd LF_yFoot = L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd RF_zFoot = Foot.RF_zsimulation_straightwalk();
	MatrixXd LF_zFoot = Foot.LF_zsimulation_straightwalk();
	MatrixXd relativeRFx = RF_xFoot.block(0, 0, RF_xFoot.rows(), sim_n) - Xcom.block(0, 0, RF_xFoot.rows(), sim_n);
	MatrixXd relativeLFx = LF_xFoot.block(0, 0, LF_xFoot.rows(), sim_n) - Xcom.block(0, 0, LF_xFoot.rows(), sim_n);
	MatrixXd relativeRFy = RF_yFoot.block(0, 0, RF_yFoot.rows(), sim_n) - Ycom.block(0, 0, RF_yFoot.rows(), sim_n);
	MatrixXd relativeLFy = LF_yFoot.block(0, 0, LF_yFoot.rows(), sim_n) - Ycom.block(0, 0, LF_yFoot.rows(), sim_n);
	BRP_Inverse_Kinematics joint;
	this->Motion5_RL = joint.BRP_RL_Simulation(relativeRFx, relativeRFy, RF_zFoot);
	this->Motion5_LL = joint.BRP_LL_Simulation(relativeLFx, relativeLFy, LF_zFoot);
}
MatrixXd Wonbin::Motions::Return_Motion5_RL() {
	return Motion5_RL;
};
MatrixXd Wonbin::Motions::Return_Motion5_LL() {

	return Motion5_LL;
};
//수정
void Wonbin::Motions::Motion6() {
	X_Com XCOM;
	Y_Com YCOM;
	Foot Foot;
	XCOM.Change_Ref_Xpos(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	MatrixXd Xcom = XCOM.XComSimulation();
	YCOM.Change_Ref_Ypos(0.045, -0.045, 0.045, -0.045, 0.045, -0.045, 0.045, -0.045, 0.045, -0.045, 0.045, -0.045);
	MatrixXd Ycom = YCOM.YComSimulation();
	Foot.Change_step(0);
	MatrixXd LF_xFoot = Foot.LF_xsimulation_straightwalk();
	MatrixXd RF_xFoot = Foot.RF_xsimulation_straightwalk();
	MatrixXd RF_yFoot = -L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd LF_yFoot = L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd RF_zFoot = Foot.RF_zsimulation_straightwalk();
	MatrixXd LF_zFoot = Foot.LF_zsimulation_straightwalk();
	MatrixXd relativeRFx = RF_xFoot.block(0, 0, RF_xFoot.rows(), sim_n) - Xcom.block(0, 0, RF_xFoot.rows(), sim_n);
	MatrixXd relativeLFx = LF_xFoot.block(0, 0, LF_xFoot.rows(), sim_n) - Xcom.block(0, 0, LF_xFoot.rows(), sim_n);
	MatrixXd relativeRFy = RF_yFoot.block(0, 0, RF_yFoot.rows(), sim_n) - Ycom.block(0, 0, RF_yFoot.rows(), sim_n);
	MatrixXd relativeLFy = LF_yFoot.block(0, 0, LF_yFoot.rows(), sim_n) - Ycom.block(0, 0, LF_yFoot.rows(), sim_n);
	BRP_Inverse_Kinematics joint;
	this->Motion6_RL = joint.BRP_RL_Simulation(relativeRFx, relativeRFy, RF_zFoot);
	this->Motion6_LL = joint.BRP_LL_Simulation(relativeLFx, relativeLFy, LF_zFoot);
}
MatrixXd Wonbin::Motions::Return_Motion6_RL() {
	return Motion6_RL;
};
MatrixXd Wonbin::Motions::Return_Motion6_LL() {
	return Motion6_LL;
};

int main()
{
	Wonbin::Motions motion;
	motion.Motion1();
	


}


