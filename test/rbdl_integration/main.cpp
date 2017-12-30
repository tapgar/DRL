/*
 * main.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: tapgar
 */

#include "DynamicModel.h"
#include "OSC_RBDL.h"

#include "mujoco.h"
#include "Visualizer.h"

int main() {

	mj_activate("/home/tapgar/.mujoco/mjkey.txt");
	char error[1000] = "Could not load binary model";
	mjModel* mj_Model = mj_loadXML("/home/tapgar/cuda-workspace/DRL/models/singleleg/singleleg.xml", 0, error, 1000);
	if (!mj_Model) {
		mju_error_s("Load model error: %s", error);
		return -1;
	}
	// Initialize mjData
	mjData* mj_Data = mj_makeData(mj_Model);

	double qpos_init[] = {0.61693030,-1.34964927,1.57217705,-1.68179016,-0.56218484};

	mju_copy(mj_Data->qpos, qpos_init, nQ);

	mj_forward(mj_Model, mj_Data);

	Visualizer* vis = new Visualizer(mj_Model, false, "remote_cassie");

	DynamicModel cassie;
	cassie.LoadModel();
	cassie.setState(mj_Data->qpos, mj_Data->qvel);

	int contactIds[] = {0, 1};
	int targetIds[] = {0, 1};

	OSC_RBDL osc = OSC_RBDL(contactIds, targetIds);
	osc.AddQDDIdx(4);

	osc.InitMatrices(&cassie);

	Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd = Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1>::Zero();
	Eigen::Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive;
	for (int i = 0; i < DOF*XDD_TARGETS; i++)
		bActive(i,0) = true;
	bActive(DOF*XDD_TARGETS,0) = false;
	bool bContact[nC] = {false, false};
	Eigen::Matrix<double, nU, 1> u = Eigen::Matrix<double, nU, 1>::Zero();

	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> x_t;
	x_t(0) = 0.079;
	x_t(1) = -0.5;
	x_t(2) = 0.0;
	x_t(3) = -0.079;
	x_t(4) = -0.5;
	x_t(5) = 0.0;

	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> xd_t = Eigen::Matrix<double, DOF*XDD_TARGETS, 1>::Zero();
	double Kpp = 100.0;
	double Kpv = 50.0;

	double freq = 0.5;
	double A = 0.3;

	double t = 0.0;
	while (true)
	{
		cassie.setState(mj_Data->qpos, mj_Data->qvel);

		Eigen::VectorXd x = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);
		Eigen::VectorXd xd = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);

		x_t(1) = x_t(4) = -0.7 + A*sin(2*M_PI*freq*t);
		xd_t(1) = xd_t(4) = A*cos(2*M_PI*freq*t);

		cassie.GetTargetPoints(&x, &xd);
		for (int i = 0; i < DOF*XDD_TARGETS; i++)
			xdd(i) = Kpp*(x_t(i) - x(i)) + Kpv*(xd_t(i) - xd(i));

		xdd(1) += -A*sin(2*M_PI*freq*t);
		xdd(4) += -A*sin(2*M_PI*freq*t);

		osc.RunPTSC(&cassie, xdd, bActive, bContact, &u);
		memcpy(mj_Data->ctrl, u.data(), nU*sizeof(double));
		mj_step(mj_Model, mj_Data);
		vis->Draw(mj_Data);

		t += 0.0005;
	}

	return 0;

}



