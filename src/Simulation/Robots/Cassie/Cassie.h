/*
 * Cassie.h
 *
 *  Created on: Nov 3, 2017
 *      Author: tapgar
 */

#ifndef Cassie_H_
#define Cassie_H_

#include <stdbool.h>
#include "mujoco.h"
#include "SharedRobotDefinitions.h"
#include "CommandInterface.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OSC.h"


#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <vector>
#include <memory>



class Cassie {

public:
	Cassie();

	void StandTest();

	void GetFootandCOM(double* x, double* h);
	void GetFootandCOM_Mordatch(double* x);

	void Update(const ROM_Policy_Struct* targ_traj, CommandInterface::OP_STATE eOpState, int nActiveIdx, double plan_time, double step_time, double* swing_targ, double swing_time);

	void GetState(double* qpos)
	{
		for (int i = 0; i < nX; i++)
			qpos[i] = mj_Data->qpos[i];
	};
	void GetCOMFeet(double* com, double* comvel, double* left, double* right);

private:

	mjModel* mj_Model;
	mjData* mj_Data;

	void StandingController();
	void WalkingController(const ROM_Policy_Struct* targ_traj, int nConIdx, double plan_time, double step_time, double* swing_targ, double swing_time);

	void RunPTSC(double* xddCOM, double* xddLeft, double* xddRight, double alphaPitch, bool* bContact, double* u);

	double FeedbackAccel(double targetPos, double targetVel, double Kp, double Kd, int siteIdx, int offset);

	void GetCOMTarget(const ROM_Policy_Struct* targ_traj, double plan_time, double* com_targ, double* com_ff);
	void GetSwingFootFF(double* cur_pos, double* cur_vel, double* targ_pt, double step_time, double* swing_targ, double swing_time, double* ff_accel);

	PD_CONTROLLER PD_COM;
	PD_CONTROLLER PD_Stance;
	PD_CONTROLLER PD_Swing;
	PD_CONTROLLER PD_Pitch;

	static constexpr double cx[] = {0.079, -0.079};

	OSC* m_pOSC;

	double m_dMass_kg;
	double m_dStiffness_Nm;

	int m_nRightSiteId;
	int m_nLeftSiteId;

	int contactSiteIds[nC];
	int targetSiteIds[XDD_TARGETS];
	int contactBodyIds[nC];

	StandingParams standParams;
	WalkingParams walkParams;

};


#endif /* Cassie_H_ */
