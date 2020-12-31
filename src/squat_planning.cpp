#include "squat_planning/code_generation_active_motor.h"
//#include <complex>
#include"Eigen/Dense"
#include<iostream>
#include <algorithm>
#include <valarray>

using namespace std;
void squat_planning ()
{
	double t_end,squat_t,delta_t,t_real,dY;
	double data_delta = 0.01;
	ExtU rtU;
	ExtY rtY;
	//rtU.Y_d = Y;
	//rtU.t_sim0 = t_start;
	//rtU.t_sim = t_s;
	//rtU.X_d = X;
	//rtU.forward_vel = vel;
	//double angle_a[800],angle_k[800],angle_h[800];
	valarray<double> angle_a[800],angle_k[800],angle_h[800]; 
	//double angular_a[800],angular_k[800],angular_h[800];
	valarray<double> angular_a[800],angular_k[800],angular_h[800];
	for (int i = 0;i <= 799;i++){  
	//double t_real;
	t_real = i * data_delta;
	rtU = {-100,2,t_real,3,1};
	t_end = rtU.X_d / rtU.forward_vel + rtU.t_sim0;
	squat_t = t_end - rtU.t_sim0;
	delta_t = rtU.t_sim-rtU.t_sim0;

	if (rtU.t_sim <= rtU.t_sim0) {
		dY = 0.0;
	}
	else if (rtU.t_sim <= t_end) {
		double a0,a1,a2,a3,a4,a5;
		a0 = 0;
		a1 = 0;
		a2 = 0;
		a3 = (20 * rtU.Y_d) * (1 / (2 * pow(squat_t,3)));
		a4 = (-30 * rtU.Y_d) * (1 / (2 * pow(squat_t,4)));
		a5 = (12 * rtU.Y_d) * (1 / (2 * pow(squat_t,5)));
		dY = a0+a1*pow(delta_t,1)+a2*pow(delta_t,2)+a3*pow(delta_t,3)+a4*pow(delta_t,4)+a5*pow(delta_t,5);
	}
	else {
		dY = rtU.Y_d;
	}
	//以上为笛卡尔轨迹规划

	double dr,rd,La,Lk,X0,Y0,k,qa0,qk0,qh0;
	dr = M_PI/180;
	rd = 180/M_PI;

	La = 188.4882;
	Lk = 222.0;
	X0 = 65.4;
	Y0 = 374.9;

	k = 0;
	qa0 = 104.0935;
	qk0 = 59.9112;
	qh0 = 105.3751;

	Eigen::Matrix<double, 2, 1> F;
	Eigen::Matrix<double, 2, 1> kcal_0;
	Eigen::Matrix<double, 2, 2> G;
	Eigen::Matrix<double, 2, 1> d_kcal;
	Eigen::Matrix<double, 3, 1> q_passive;

	kcal_0 << (qa0 - 0.213 * dY) * dr,\
			 (0.232 * dY + qk0) * dr;

	F << (La * cos(kcal_0(0,0)) + Lk * cos(kcal_0(1,0))) - X0,\
	     (La * sin(kcal_0(0,0)) + Lk * sin(kcal_0(1,0))) - (Y0 + dY);

	while(F.norm()>1E-10){
		F << (La * cos(kcal_0(0,0)) + Lk * cos(kcal_0(1,0))) - X0,\
		 (La * sin(kcal_0(0,0)) + Lk * sin(kcal_0(1,0))) - (Y0 + dY);
		G << -La * sin(kcal_0(0,0)),\
		 -Lk * sin(kcal_0(1,0)),\
		  La * cos(kcal_0(0,0)),\
		  Lk * cos(kcal_0(1,0));

		d_kcal = -G.inverse() * F;
		kcal_0 += d_kcal;
		k++;
	   if (k >= 100)
	   break;
	}
	q_passive << kcal_0(0,0) * rd,\
				 kcal_0(1,0) * rd,\
				 0;
	//以上为逆运动学解析

	double qm10,q_delta,qc10,qs10,qf10,qc1,delta_qa;
	qm10 = 204.1062;
	q_delta = 14.3573;
	qc10 = 89.7362;
	qs10 = 256.8924;
	qf10 = -6.6292;
	qc1 = q_passive(0,0)-q_delta;
    delta_qa = qc1-qc10;

	double Lm1,Ls1,Lf1,Lc1;
	Lm1 = 40;
	Ls1 = 112;
	Lf1 = 61.7134;
	Lc1 = 132.5449;

	Eigen::Matrix<double, 2, 1> lcal_0;
	Eigen::Matrix<double, 2, 1> d_lcal;

	lcal_0 << (qm10 - delta_qa * 1.55) * dr,\
			  (qs10 + delta_qa * 1.76) * dr;
	k = 0;
	F << Lm1 * cos(lcal_0(0,0))+Ls1 * cos(lcal_0(1,0))+Lf1 * cos(qf10*dr)+Lc1 * cos(qc1*dr),\
		 Lm1 * sin(lcal_0(0,0))+Ls1 * sin(lcal_0(1,0))+Lf1 * sin(qf10*dr)+Lc1 * sin(qc1*dr);
	while(F.norm()>1E-10){
		F << Lm1 * cos(lcal_0(0,0))+Ls1 * cos(lcal_0(1,0))+Lf1 * cos(qf10*dr)+Lc1 * cos(qc1*dr),\
		 	 Lm1 * sin(lcal_0(0,0))+Ls1 * sin(lcal_0(1,0))+Lf1 * sin(qf10*dr)+Lc1 * sin(qc1*dr);
		G << -Lm1 * sin(lcal_0(0,0)),\
		 	 -Ls1 * sin(lcal_0(1,0)),\
		 	  Lm1 * cos(lcal_0(0,0)),\
		 	  Ls1 * cos(lcal_0(1,0));

		d_lcal = -G.inverse() * F;
		lcal_0 += d_lcal;
		k++;
	   if (k >= 100)
	   break;
	}	 

	double qm20,qc20,qf20,qf2,delta_qk;
	qm20 = -122.7125;
	qc20 = -34.0795;
	qf20 = 167.8;
	qf2 = qf20 + q_passive(0,0)-qa0;
    delta_qk = q_passive(1,0);

	double Lm2,Ls2,Lf2,Lc2;
	Lm2 = 193;
	Ls2 = 222;
	Lf2 = 76.6775;
	Lc2 = 82;

	Eigen::Matrix<double, 2, 1> jcal_0;
	Eigen::Matrix<double, 2, 1> d_jcal;

	jcal_0 << (qm20 - delta_qk) * dr,\
			  (qc20 - delta_qk) * dr;
	k = 0;
	F << Lm2 * cos(jcal_0(0,0))+Ls2 * cos(q_passive(1,0)*dr)+Lf2 * cos(qf2*dr)+Lc2 * cos(jcal_0(1,0)),\
		 Lm2 * sin(jcal_0(0,0))+Ls2 * sin(q_passive(1,0)*dr)+Lf2 * sin(qf2*dr)+Lc2 * sin(jcal_0(1,0));
	while(F.norm()>1E-10){
		F << Lm2 * cos(jcal_0(0,0))+Ls2 * cos(q_passive(1,0)*dr)+Lf2 * cos(qf2*dr)+Lc2 * cos(jcal_0(1,0)),\
		 	 Lm2 * sin(jcal_0(0,0))+Ls2 * sin(q_passive(1,0)*dr)+Lf2 * sin(qf2*dr)+Lc2 * sin(jcal_0(1,0));
		G << -Lm2 * sin(jcal_0(0,0)),\
		 	 -Lc2 * sin(jcal_0(1,0)),\
		 	  Lm2 * cos(jcal_0(0,0)),\
		 	  Lc2 * cos(jcal_0(1,0));

		d_jcal = -G.inverse() * F;
		jcal_0 += d_jcal;
		k++;
	   if (k >= 100)
	   break;
	}	 
	rtY.ankle = lcal_0(0,0)*rd-qm10-delta_qa;
	rtY.knee = jcal_0(0,0)*rd-qm20;
	rtY.hip = 0;

	angle_a[i] = rtY.ankle;
	angle_k[i] = rtY.knee;
	angle_h[i] = rtY.hip;
	//cout << angle_a[i] << endl;
    //cout << angle_k[i] << endl;
    //cout << angle_h[i] <<endl;
}
	for (int vi = 0;vi <= 799;vi++){  
		t_real = data_delta;
		if (vi == 799 ){
			angular_a[vi] = 0;
			angular_k[vi] = 0;
			angular_h[vi] = 0;
		}
		else{
			angular_a[vi] = (angle_a[vi+1]-angle_a[vi])/t_real;
			angular_k[vi] = (angle_k[vi+1]-angle_k[vi])/t_real;
			angular_h[vi] = (angle_h[vi+1]-angle_h[vi])/t_real;
		}

	//cout << angular_a[vi] << endl;
    //cout << angular_k[vi] << endl;
    //cout << angular_h[vi] <<endl;
	}
	/*angle_a = 100 * angle_a;
	angle_k = 100 * angle_k;
	angle_h = 100 * angle_h;

	float Angular_a[800];
	int **a = (int**)malloc(sizeof(int*)*14);
	if(a){
		for(int i = 0; i<14; i++){
			a[i] = (int*)malloc(sizeof(int)*800);
		}
	}
	copy(begin(angular_a),end(angular_a),begin(a[0]));*/
	getchar();
}

