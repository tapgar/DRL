/*
 * Cassie.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: tapgar
 */



#include "Cassie.h"
#include <iostream>

using namespace Eigen;
using namespace std;

constexpr double Cassie::cx[];

Cassie::Cassie() {

	mj_activate("/home/tapgar/.mujoco/mjkey.txt");
	char error[1000] = "Could not load binary model";
	mj_Model = mj_loadXML("/home/tapgar/cuda-workspace/DRL/models/cassie/cassie3d_stiff.xml", 0, error, 1000);
	if (!mj_Model) {
		mju_error_s("Load model error: %s", error);
		return;
	}

	// Initialize mjData
	mj_Data = mj_makeData(mj_Model);

	m_dMass_kg = mj_getTotalmass(mj_Model);
	m_dStiffness_Nm = mj_Model->jnt_stiffness[mj_name2id(mj_Model, mjOBJ_JOINT, "right_spring")];

	double qpos_init[] = {0.0, 0.0, 0.939, 1.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.68111815, -1.40730357, 1.62972042, -1.77611107, -0.61968407,
			0.0, 0.0, 0.68111815, -1.40730353, 1.62972043, -1.77611107, -0.61968402};

	mju_copy(mj_Data->qpos, qpos_init, nQ+1);

	mj_forward(mj_Model, mj_Data);

	m_nRightSiteId = mj_name2id(mj_Model, mjOBJ_SITE, "right_foot_pt");
	m_nLeftSiteId = mj_name2id(mj_Model, mjOBJ_SITE, "left_foot_pt");

	targetSiteIds[0] = mj_name2id(mj_Model, mjOBJ_SITE, "body_center");
	targetSiteIds[1] = mj_name2id(mj_Model, mjOBJ_SITE, "left_contact_front");
	targetSiteIds[2] = mj_name2id(mj_Model, mjOBJ_SITE, "left_contact_rear");
	targetSiteIds[3] = mj_name2id(mj_Model, mjOBJ_SITE, "right_contact_front");
	targetSiteIds[4] = mj_name2id(mj_Model, mjOBJ_SITE, "right_contact_rear");

	contactSiteIds[0] = mj_name2id(mj_Model, mjOBJ_SITE, "left_contact_front");
	contactSiteIds[1] = mj_name2id(mj_Model, mjOBJ_SITE, "left_contact_rear");
	contactSiteIds[2] = mj_name2id(mj_Model, mjOBJ_SITE, "right_contact_front");
	contactSiteIds[3] = mj_name2id(mj_Model, mjOBJ_SITE, "right_contact_rear");

	PD_COM.Kp = 100.0;
	PD_COM.Kd = 20.0;
	PD_Swing.Kp = 0.0;//100.0;
	PD_Swing.Kd = 0.0;//5.0;
	PD_Stance.Kp = 100.0;
	PD_Stance.Kd = 30.0;
	PD_Pitch.Kp = 100.0;
	PD_Pitch.Kd = 20.0;

	contactBodyIds[0] = 0;
	contactBodyIds[1] = 0;
	contactBodyIds[2] = 1;
	contactBodyIds[3] = 1;

	//initialize operational state control
	m_pOSC = new OSC(contactSiteIds, targetSiteIds, contactBodyIds);
	m_pOSC->AddQDDIdx(3); //roll regulation
	m_pOSC->AddQDDIdx(4); //pitch regulation
	m_pOSC->AddQDDIdx(5); //yaw regulation
	m_pOSC->AddQDDIdx(11); //ankle regulation
	m_pOSC->AddQDDIdx(18); //ankle regulation
	m_pOSC->InitMatrices(mj_Model);

}

double Cassie::FeedbackAccel(double targetPos, double targetVel, double Kp, double Kd, int siteIdx, int offset)
{
	double accel = 0.0;

	double pos = mj_Data->site_xpos[siteIdx*3 + offset];

	mjtNum cvel[6];
	mj_objectVelocity(mj_Model, mj_Data, mjOBJ_SITE, siteIdx, cvel, 0);

	accel = Kp*(targetPos - pos) + Kd*(targetVel - cvel[3+offset]);

	return accel;
}

void Cassie::StandingController()
{
	Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive;
	for (int i = 0; i < DOF*XDD_TARGETS+QDD_TARGETS; i++)
		bActive(i,0) = true;

	bActive(DOF*XDD_TARGETS+3,0) = false;
	bActive(DOF*XDD_TARGETS+4,0) = false;

	bool bDesiredContact[] = {true, true, true, true};

	Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd = Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1>::Zero();

	double targBz = standParams.m_dTargetZPos_m;
	double targBy = 0.0;
	double targBx = 0.0;

	double count = 0.0;
	for (int i = 1; i < 5; i++)
	{
		if (bDesiredContact[i-1])
		{
			targBx += mj_Data->site_xpos[targetSiteIds[i]*3];
			targBy += mj_Data->site_xpos[targetSiteIds[i]*3+1];
			count += 1.0;
		}
	}
	targBx /= count;
	targBy /= count;


	xdd(0,0) = PD_COM.Kp*(targBx - mj_Data->qpos[0]) - PD_COM.Kd*mj_Data->qvel[0];
	xdd(1,0) = PD_COM.Kp*(targBy - mj_Data->qpos[1]) - PD_COM.Kd*mj_Data->qvel[1];
	xdd(2,0) = PD_COM.Kp*(targBz - mj_Data->qpos[2]) - PD_COM.Kd*mj_Data->qvel[2];

	for (int i = 0; i < nC; i++)
	{
		xdd(3+DOF*i,0) = FeedbackAccel(0.0,0.0,0.0,PD_Stance.Kd,targetSiteIds[1+i],0);
		xdd(4+DOF*i,0) = FeedbackAccel(0.0,0.0,0.0,PD_Stance.Kd,targetSiteIds[1+i],1);
		xdd(5+DOF*i,0) = FeedbackAccel(-5e-3,0.0,PD_Stance.Kp,PD_Stance.Kd,targetSiteIds[1+i],2);
	}

	//make x point accels equal
	xdd(3,0) = (xdd(3,0) + xdd(6,0))/2.0;
	xdd(6,0) = xdd(3,0);
	xdd(9,0) = (xdd(12,0) + xdd(9,0))/2.0;
	xdd(12,0) = xdd(9,0);

	mjtNum targq[nQ+1];
	mju_zero(targq,nQ+1);
	targq[3] = 1.0;
	mjtNum qdiff[nQ];
	mju_zero(qdiff,nQ);
	mj_differentiatePos(mj_Model, qdiff, 1.0, targq, mj_Data->qpos);


	xdd(15,0) = PD_Pitch.Kp*(0.0 - qdiff[3]) - PD_Pitch.Kd*mj_Data->qvel[3];
	xdd(16,0) = PD_Pitch.Kp*(standParams.m_dTargetPitch_rad - qdiff[4]) - PD_Pitch.Kd*mj_Data->qvel[4];
	xdd(17,0) = PD_Pitch.Kp*(standParams.m_dTargetYaw_rad - qdiff[5]) - PD_Pitch.Kd*mj_Data->qvel[5];

	Matrix<double, nU, 1> u = Matrix<double, nU, 1>::Zero();
	Matrix<double, DOF*nC, 1> f = Matrix<double, DOF*nC, 1>::Zero();
	Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd_e;

	bool bInContact[nC];
	for (int i = 0; i < nC; i++)
		bInContact[i] = bDesiredContact[i];

	double residual = 0.0;
	Matrix<double, nQ, 1> qdd_e = Matrix<double, nQ, 1>::Zero();

	m_pOSC->RunPTSC(mj_Model, mj_Data, xdd, bActive, bInContact, &u, &f, &xdd_e, &qdd_e, &residual);

	mju_copy(mj_Data->ctrl, u.data(), nU);
}

void Cassie::Update(const ROM_Policy_Struct* targ_traj, CommandInterface::OP_STATE eOpState, int nActiveIdx, double plan_time, double step_time, double* swing_targ, double swing_time) {

	switch (eOpState)
	{
//	case CommandInterface::Walking_DS:
//	case CommandInterface::Walking_SS_Left:
//	case CommandInterface::Walking_SS_Right:
	case CommandInterface::Idle:
		standParams.m_dTargetZPos_m = m_dNominalZTarget_m;
		standParams.m_dTargetYaw_rad = standParams.m_dTargetPitch_rad = 0.0;
		StandingController();
		break;

	case CommandInterface::Standing:
		StandingController();
		break;

	case CommandInterface::Walking_DS:
	case CommandInterface::Walking_SS_Left:
	case CommandInterface::Walking_SS_Right:
		WalkingController(targ_traj, nActiveIdx, plan_time, step_time, swing_targ, swing_time);

		break;
	}

	mj_step(mj_Model, mj_Data);
}


void Cassie::GetCOMTarget(const ROM_Policy_Struct* targ_traj, double plan_time, double* com_targ, double* com_ff)
{
	int N = int(plan_time / targ_traj->dt_c);
	if (N > MAX_TRAJ_PTS-1)
	{
//		printf("youre fucked!\n");
		N = MAX_TRAJ_PTS-2;
	}

	for (int i = 0; i < 4; i++)
		com_targ[i] = targ_traj->com_traj[N].com[i];
	for (int i = 0; i < 4; i++)
		com_targ[i+4] = (targ_traj->com_traj[N+1].com[i] - targ_traj->com_traj[N-1].com[i])/(2*targ_traj->dt_c);
	for (int i = 0; i < 4; i++)
		com_ff[i] = targ_traj->com_traj[N].com_xdd[i];

//	for (int i = 0; i < 3; i++)
//	{
//		printf("%f\t%f\t%f\n", com_targ[i], com_targ[i+3], com_ff[i]);
//	}
}

void Cassie::GetSwingFootFF(double* cur_pos, double* cur_vel, double* targ_pt, double step_time, double* swing_targ, double swing_time, double* ff_accel)
{
	double t, T;
	Matrix2d A = Matrix2d::Zero();
	Vector2d b = Vector2d::Zero();
	Vector2d ab = Vector2d::Zero();

	double* targ = targ_pt;
	double vel_targ[] = {0.0, 0.0, 0.0};
	if (step_time < swing_time/2.0)
	{
		targ = swing_targ;
		T = (swing_time/2.0) - step_time;
		for (int i = 0; i < 2; i++)
		{
			double dT = swing_time;
			double dX = (targ_pt[i] - swing_targ[i])*2.0;
			double const_vel = dX/dT;
			vel_targ[i] = 2.0*const_vel; //const accel/decel profile
		}
	}
	else
	{
		T = swing_time - step_time;
	}


	//you may need to make a seperate region near the end of the swing phase where T gets small
	t = 0.0;

	A << pow(T,2.0)/3.0, pow(T,2.0)/6.0, T/2, T/2;

	for (int i = 0; i < 3; i++)
	{
		double y0, yd0, yT, ydT;
		y0 = cur_pos[i];
		yd0 = cur_vel[i];

		yT = targ[i];
		ydT = vel_targ[i];
		b << yT - y0 - yd0*T, ydT - yd0;
		ab = pseudoinverse(A)*b;
		ff_accel[i] = ab(0)*(1-t/T) + ab(1)*t/T;
	}

//	printf("swing foot\n");
//	for (int i = 0; i < 3; i++)
//		printf("%f\t%f\t%f\t%f\t%f\n", cur_pos[i], cur_vel[i], targ_pt[i], swing_targ[i], ff_accel[i]);
}

void Cassie::WalkingController(const ROM_Policy_Struct* targ_traj, int nConIdx, double plan_time, double step_time, double* swing_targ, double swing_time)
{
	Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive;
	for (int i = 0; i < DOF*XDD_TARGETS+QDD_TARGETS; i++)
		bActive(i,0) = true;

	bActive(DOF*XDD_TARGETS+3,0) = false;
	bActive(DOF*XDD_TARGETS+4,0) = false;

	bool bDesiredContact[] = {true, true, true, true};

	Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd = Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1>::Zero();

	double com_targ[8], com_ff[4];
	GetCOMTarget(targ_traj, plan_time, com_targ, com_ff);

	for (int i = 0; i < 3; i++)
	{
		xdd(i,0) = PD_COM.Kp*(com_targ[i] - mj_Data->qpos[i]) + PD_COM.Kd*(com_targ[i+4] - mj_Data->qvel[i]);
		xdd(i,0) += com_ff[i];
	}

	const double* targ_left = targ_traj->con_sched[nConIdx].left;
	const double* targ_right = targ_traj->con_sched[nConIdx].right;
	mjtNum cvel[6];
	if (targ_traj->con_sched[nConIdx].con_state == SS_Left)
	{
		bDesiredContact[2] = bDesiredContact[3] = false;

		//left stance
		for (int i = 0; i < 2; i++)
		{
			xdd(3+DOF*i,0) = FeedbackAccel(0.0,0.0,0.0,PD_Stance.Kd,targetSiteIds[1+i],0);
			xdd(4+DOF*i,0) = FeedbackAccel(0.0,0.0,0.0,PD_Stance.Kd,targetSiteIds[1+i],1);
			xdd(5+DOF*i,0) = FeedbackAccel(-5e-3,0.0,PD_Stance.Kp,PD_Stance.Kd,targetSiteIds[1+i],2);
		}
		for (int i = 2; i < 4; i++)
		{
			double ff_accel[3];
			double cur_pos[] = {mj_Data->site_xpos[targetSiteIds[1+i]*3], mj_Data->site_xpos[targetSiteIds[1+i]*3+1], mj_Data->site_xpos[targetSiteIds[1+i]*3+2]};
			double targ_pos[] = {targ_right[0] + cx[i-2]*cos(targ_right[3]), targ_right[1] + cx[i-2]*sin(targ_right[3]), targ_right[2]};
			double apex_targ[] = {swing_targ[0] + cx[i-2]*cos(targ_right[3]), swing_targ[1] + cx[i-2]*sin(targ_right[3]), swing_targ[2]};

			mj_objectVelocity(mj_Model, mj_Data, mjOBJ_SITE, targetSiteIds[1+i], cvel, 0);

			GetSwingFootFF(cur_pos, &(cvel[3]), targ_pos, step_time, apex_targ, swing_time, ff_accel);
			xdd(3+DOF*i,0) = ff_accel[0];
			xdd(4+DOF*i,0) = ff_accel[1];
			xdd(5+DOF*i,0) = ff_accel[2];
		}
		//make x point accels equal
		xdd(3,0) = (xdd(3,0) + xdd(6,0))/2.0;
		xdd(6,0) = xdd(3,0);
	}
	else if (targ_traj->con_sched[nConIdx].con_state == SS_Right)
	{
		bDesiredContact[0] = bDesiredContact[1] = false;

		for (int i = 0; i < 2; i++)
		{
			double ff_accel[3];
			double cur_pos[] = {mj_Data->site_xpos[targetSiteIds[1+i]*3], mj_Data->site_xpos[targetSiteIds[1+i]*3+1], mj_Data->site_xpos[targetSiteIds[1+i]*3+2]};
			double targ_pos[] = {targ_left[0] + cx[i]*cos(targ_left[3]), targ_left[1] + cx[i]*sin(targ_left[3]), targ_left[2]};
			double apex_targ[] = {swing_targ[0] + cx[i]*cos(targ_left[3]), swing_targ[1] + cx[i]*sin(targ_left[3]), swing_targ[2]};

			mj_objectVelocity(mj_Model, mj_Data, mjOBJ_SITE, targetSiteIds[1+i], cvel, 0);

			GetSwingFootFF(cur_pos, &cvel[3], targ_pos, step_time, apex_targ, swing_time, ff_accel);
			xdd(3+DOF*i,0) = ff_accel[0];
			xdd(4+DOF*i,0) = ff_accel[1];
			xdd(5+DOF*i,0) = ff_accel[2];
		}
		//right stance
		for (int i = 2; i < nC; i++)
		{
			xdd(3+DOF*i,0) = FeedbackAccel(0.0,0.0,0.0,PD_Stance.Kd,targetSiteIds[1+i],0);
			xdd(4+DOF*i,0) = FeedbackAccel(0.0,0.0,0.0,PD_Stance.Kd,targetSiteIds[1+i],1);
			xdd(5+DOF*i,0) = FeedbackAccel(-5e-3,0.0,PD_Stance.Kp,PD_Stance.Kd,targetSiteIds[1+i],2);
		}
		xdd(9,0) = (xdd(12,0) + xdd(9,0))/2.0;
		xdd(12,0) = xdd(9,0);
	}
	else
	{
		//both stance
		for (int i = 0; i < nC; i++)
		{
			xdd(3+DOF*i,0) = FeedbackAccel(0.0,0.0,0.0,PD_Stance.Kd,targetSiteIds[1+i],0);
			xdd(4+DOF*i,0) = FeedbackAccel(0.0,0.0,0.0,PD_Stance.Kd,targetSiteIds[1+i],1);
			xdd(5+DOF*i,0) = FeedbackAccel(-5e-3,0.0,PD_Stance.Kp,PD_Stance.Kd,targetSiteIds[1+i],2);
		}
		//make x point accels equal
		xdd(3,0) = (xdd(3,0) + xdd(6,0))/2.0;
		xdd(6,0) = xdd(3,0);
		xdd(9,0) = (xdd(12,0) + xdd(9,0))/2.0;
		xdd(12,0) = xdd(9,0);
	}

	mjtNum targq[nQ+1];
	mju_zero(targq,nQ+1);
	targq[3] = 1.0;
	mjtNum qdiff[nQ];
	mju_zero(qdiff,nQ);
	mj_differentiatePos(mj_Model, qdiff, 1.0, targq, mj_Data->qpos);


	xdd(15,0) = PD_Pitch.Kp*(0.0 - qdiff[3]) - PD_Pitch.Kd*mj_Data->qvel[3];
	xdd(16,0) = PD_Pitch.Kp*(standParams.m_dTargetPitch_rad - qdiff[4]) - PD_Pitch.Kd*mj_Data->qvel[4];
	xdd(17,0) = PD_Pitch.Kp*(standParams.m_dTargetYaw_rad - qdiff[5]) - PD_Pitch.Kd*mj_Data->qvel[5];
//	xdd(17,0) = PD_Pitch.Kp*(com_targ[3] - qdiff[5]) + PD_Pitch.Kd*(com_targ[7] - mj_Data->qvel[5]) + com_ff[3];
	printf("Targets: %f,%f,%f\t\tMeasured: %f,%f\n", com_targ[3], com_targ[7], com_ff[3], qdiff[5], mj_Data->qvel[5]);


	Matrix<double, nU, 1> u = Matrix<double, nU, 1>::Zero();
	Matrix<double, DOF*nC, 1> f = Matrix<double, DOF*nC, 1>::Zero();
	Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd_e;

	bool bInContact[nC];
	for (int i = 0; i < nC; i++)
		bInContact[i] = bDesiredContact[i];

	double residual = 0.0;
	Matrix<double, nQ, 1> qdd_e = Matrix<double, nQ, 1>::Zero();

	m_pOSC->RunPTSC(mj_Model, mj_Data, xdd, bActive, bInContact, &u, &f, &xdd_e, &qdd_e, &residual);

	mju_copy(mj_Data->ctrl, u.data(), nU);

}

void Cassie::GetFootandCOM(double* x, double* zpos)
{
	x[0] = mj_Data->qpos[0];
	x[1] = mj_Data->qpos[1];
	x[2] = mj_Data->qvel[0];
	x[3] = mj_Data->qvel[1];
	x[4] = (mj_Data->site_xpos[targetSiteIds[1]*3] + mj_Data->site_xpos[targetSiteIds[2]*3])/2.0;
	x[5] = (mj_Data->site_xpos[targetSiteIds[1]*3+1] + mj_Data->site_xpos[targetSiteIds[2]*3+1])/2.0;
	x[6] = 1.0;
	x[7] = 0.0;
	x[8] = (mj_Data->site_xpos[targetSiteIds[3]*3] + mj_Data->site_xpos[targetSiteIds[4]*3])/2.0;
	x[9] = (mj_Data->site_xpos[targetSiteIds[3]*3+1] + mj_Data->site_xpos[targetSiteIds[4]*3+1])/2.0;
	x[10] = 1.0;
	x[11] = 0.0;
	*zpos = mj_Data->qpos[2];

	int feetIds[] = {targetSiteIds[1],targetSiteIds[2],targetSiteIds[4],targetSiteIds[3]};
	for (int i = 0; i < nC; i++)
	{
		for (int j = 0; j < DOF; j++)
			printf("%f\t",mj_Data->site_xpos[feetIds[i]*3 + j]);
		printf("\n");
	}
}

void Cassie::GetFootandCOM_Mordatch(double* x)
{
	x[0] = mj_Data->qpos[0];
	x[1] = mj_Data->qpos[1];
	x[2] = mj_Data->qpos[2];

	mjtNum targq[nQ+1];
	mju_zero(targq,nQ+1);
	targq[3] = 1.0;
	mjtNum qdiff[nQ];
	mju_zero(qdiff,nQ);
	mj_differentiatePos(mj_Model, qdiff, 1.0, targq, mj_Data->qpos);
	x[3] = -qdiff[5];

	x[4] = mj_Data->qvel[0];
	x[5] = mj_Data->qvel[1];
	x[6] = mj_Data->qvel[2];
	x[7] = mj_Data->qvel[5];

	x[8] = (mj_Data->site_xpos[targetSiteIds[1]*3] + mj_Data->site_xpos[targetSiteIds[2]*3])/2.0;
	x[9] = (mj_Data->site_xpos[targetSiteIds[1]*3+1] + mj_Data->site_xpos[targetSiteIds[2]*3+1])/2.0;
	x[10] = 0.0;
	x[11] = (mj_Data->site_xpos[targetSiteIds[3]*3] + mj_Data->site_xpos[targetSiteIds[4]*3])/2.0;
	x[12] = (mj_Data->site_xpos[targetSiteIds[3]*3+1] + mj_Data->site_xpos[targetSiteIds[4]*3+1])/2.0;
	x[13] = 0.0;

}

void Cassie::GetCOMFeet(double* com, double* comvel, double* left, double* right)
{
	for (int i = 0; i < 3; i++)
	{
		com[i] = mj_Data->qpos[i];
		comvel[i] = mj_Data->qvel[i];
	}
	mjtNum targq[nQ+1];
	mju_zero(targq,nQ+1);
	targq[3] = 1.0;
	mjtNum qdiff[nQ];
	mju_zero(qdiff,nQ);
	mj_differentiatePos(mj_Model, qdiff, 1.0, targq, mj_Data->qpos);
	com[3] = -qdiff[5];

	comvel[3] = mj_Data->qvel[3];

	left[0] = (mj_Data->site_xpos[targetSiteIds[1]*3] + mj_Data->site_xpos[targetSiteIds[2]*3])/2.0;
	left[1] = (mj_Data->site_xpos[targetSiteIds[1]*3+1] + mj_Data->site_xpos[targetSiteIds[2]*3+1])/2.0;
	left[2] = (mj_Data->site_xpos[targetSiteIds[1]*3+2] + mj_Data->site_xpos[targetSiteIds[2]*3+2])/2.0;
	left[3] = 0.0;
	right[0] = (mj_Data->site_xpos[targetSiteIds[3]*3] + mj_Data->site_xpos[targetSiteIds[4]*3])/2.0;
	right[1] = (mj_Data->site_xpos[targetSiteIds[3]*3+1] + mj_Data->site_xpos[targetSiteIds[4]*3+1])/2.0;
	right[2] = (mj_Data->site_xpos[targetSiteIds[3]*3+2] + mj_Data->site_xpos[targetSiteIds[4]*3+2])/2.0;
	right[3] = 0.0;
}
