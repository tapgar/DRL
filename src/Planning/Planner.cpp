/*
 * Planner.cpp
 *
 *  Created on: Dec 8, 2017
 *      Author: tapgar
 */

#include "Planner.h"
#include <unistd.h>

using namespace std;

Planner::Planner() {
	// TODO Auto-generated constructor stub
	opt = new MPC_OPTIONS();
	mpc.Init();

	targ_traj.dt_c = 0.0;
	targ_traj.step_height = 0.15;
	ROM_TrajPt_Struct null;
	for (int i = 0; i < MAX_TRAJ_PTS; i++)
		targ_traj.com_traj.push_back(null);
	ContactInfo_Struct null_con;
	for (int i = 0; i < MAX_CON_SWITCH; i++)
		targ_traj.con_sched.push_back(null_con);

	usr_cmd.ds_perc = 0.2;
	usr_cmd.step_time = 0.25;
	usr_cmd.step_height = 0.15;
	usr_cmd.heading = 0.0;
	usr_cmd.xT.x_m = 0.0;
	usr_cmd.xT.y_m = 0.0;
	usr_cmd.xT.z_m = 0.0;
#if LOGGING
	fileTraj.open("traj.csv", std::ofstream::out);
#endif
}

Planner::~Planner() {
	// TODO Auto-generated destructor stub
}

void Planner::Run(list<CommandInterface::MESSAGE_TYPE>* robot_msg, CommHandler* pRobotComm, list<CommandInterface::MESSAGE_TYPE>* ui_cmd_list, CommHandler* pUIComm)
{
	bool bNewCommand = CheckForNewCommands(ui_cmd_list, pUIComm);
	bool bUpdatedState = CheckRobotState(robot_msg, pRobotComm);

	if (bUpdatedState)
	{
		//save run_count into traj
		targ_traj.run_count = rcv_run_count;
		bool bSuccess = Update();
		if (bSuccess)
		{
			SendRobotUpdate(pRobotComm);
			SendUIUpdate(pUIComm);
#if LOGGING
			LogTrajectory();
#endif
		}
		pUIComm->SendState(qpos, com, com_vel, left, right, eOpState, rcv_run_count);
	}
}


void Planner::SendUIUpdate(CommHandler* pComm)
{
	//get every 10 points
	mpc.GetSolution(&targ_traj, 0.01);
	pComm->SendTrajectory(&targ_traj, 1);
}

void Planner::SendRobotUpdate(CommHandler* pComm)
{
	mpc.GetSolution(&targ_traj, 0.0005);
	targ_traj.step_height = usr_cmd.step_height;
	pComm->SendTrajectory(&targ_traj, 1);
}

bool Planner::Update() {

	//step timing.... target pos... etc

	for (int i = 0; i < 4; i++)
	{
		opt->x0[i] = com[i];
		opt->x0[i+4] = com_vel[i];
	}
	int idx[] = {0, 1, 3};
	for (int i = 0; i < 3; i++)
	{
		opt->x0[i+8] = left[idx[i]];
		opt->x0[i+11] = right[idx[i]];
	}

	opt->xT[0] = opt->x0[0] + usr_cmd.xT.x_m;
	opt->xT[1] = opt->x0[1] + usr_cmd.xT.y_m;
	opt->xT[2] = opt->x0[2] + usr_cmd.xT.z_m;
	opt->xT[3] = usr_cmd.heading;//atan2(opt->xT[1]-opt->x0[1],opt->xT[0]-opt->x0[0]);

	//if the contact schedule doesn't change from previous then a warmstart is simple
	bool bWarmStart = UpdateContactSchedule();

	mpc.SetProblem(opt, bWarmStart);

	bool bSuccess = mpc.Run();

	//	if (!bSuccess)
	//		usleep(5e5);

	return bSuccess;

}

bool Planner::CompareOpPhase(CommandInterface::OP_STATE op_state, PHASE_STATE con_state)
{
	if (op_state == CommandInterface::Walking_DS && con_state == Double)
		return true;
	if (op_state == CommandInterface::Walking_SS_Left && con_state == SS_Left)
		return true;
	if (op_state == CommandInterface::Walking_SS_Right && con_state == SS_Right)
		return true;
	return false;
}

bool Planner::UpdateContactSchedule()
{
	static uint32_t last_rcv_run_count = 0;

	bool bWarmStart = false;

	double step_time = usr_cmd.step_time;
	double ds_perc = usr_cmd.ds_perc;
	opt->num_phases = 17;

	bool bLastLeft = true;
	if (eOpState == CommandInterface::Idle || eOpState == CommandInterface::Standing)
	{
		//create new plan
		for (int i = 0; i < opt->num_phases/2; i++)
		{
			opt->phase[i*2].eType = Double;
			if (i == 0)
				opt->phase[i*2].T = 0.2;
			else
				opt->phase[i*2].T = ds_perc*step_time;

			if (bLastLeft)
				opt->phase[i*2+1].eType = SS_Right;
			else
				opt->phase[i*2+1].eType = SS_Left;
			opt->phase[i*2+1].T = (1.0 - ds_perc)*step_time;
			bLastLeft = !bLastLeft;
		}
		if (opt->num_phases % 2)
		{
			opt->phase[opt->num_phases-1].eType = Double;
			opt->phase[opt->num_phases-1].T = 0.2;
		}
	}
	else {

		//		printf("before:\n");
		//		for (int i = 0; i < opt->num_phases; i++)
		//			{
		//				printf("%d\t%f\n", opt->phase[i].eType, opt->phase[i].T);
		//			}
		//otherwise phases have been set before
		double phaseRunTime_s = double(rcv_run_count - last_rcv_run_count)*0.0005;
		double dT = 0.0;
		int nActiveIdx = 0;
		double deltaT = 0.0;
		for (int i = 0; i < opt->num_phases; i++)
		{
			dT += opt->phase[i].T;
			if (phaseRunTime_s < dT)
			{
				nActiveIdx = i;
				deltaT = dT - phaseRunTime_s;
				break;
			}
		}

		//		printf("%d\t%d\t%f\t%d\t%f\n", last_rcv_run_count, rcv_run_count, phaseRunTime_s, nActiveIdx, deltaT);

		if (nActiveIdx > 0)
		{
			int nStartIdx = 0;
			//nActiveIdx shift
			for (int i = 0; i < opt->num_phases-nActiveIdx; i++)
				opt->phase[i] = opt->phase[i+nActiveIdx];

			if (opt->phase[0].eType == SS_Left || opt->phase[0].eType == SS_Right)
			{
				bLastLeft = (opt->phase[0].eType == SS_Left);
				opt->num_phases = 16;
				//create new plan
				for (int i = 1; i < opt->num_phases/2; i++)
				{
					opt->phase[i*2+1].eType = Double;
					opt->phase[i*2+1].T = ds_perc*step_time;

					if (bLastLeft)
						opt->phase[i*2].eType = SS_Right;
					else
						opt->phase[i*2].eType = SS_Left;
					opt->phase[i*2].T = (1.0 - ds_perc)*step_time;
					bLastLeft = !bLastLeft;
				}
			}
			else
			{
				bLastLeft = !(opt->phase[1].eType != SS_Left);
				opt->num_phases = 17;
				for (int i = 1; i < opt->num_phases/2; i++)
				{
					opt->phase[i*2].eType = Double;
					opt->phase[i*2].T = ds_perc*step_time;

					if (bLastLeft)
						opt->phase[i*2+1].eType = SS_Right;
					else
						opt->phase[i*2+1].eType = SS_Left;
					opt->phase[i*2+1].T = (1.0 - ds_perc)*step_time;
					bLastLeft = !bLastLeft;
				}
			}
			opt->phase[opt->num_phases-1].eType = Double;
			opt->phase[opt->num_phases-1].T = 0.2;
		}
		else
			bWarmStart = true;

		opt->phase[0].T = deltaT;
	}

	//	for (int i = 0; i < opt->num_phases; i++)
	//	{
	//		printf("%d\t%f\n", opt->phase[i].eType, opt->phase[i].T);
	//	}

	last_rcv_run_count = rcv_run_count;

}

bool Planner::CheckForNewCommands(list<CommandInterface::MESSAGE_TYPE>* cmd_list, CommHandler* pComm)
{
	bool bNewMessage = false;

	mtx.lock();
	while (!cmd_list->empty())
	{
		CommandInterface::MESSAGE_TYPE cmd = cmd_list->front();
		if (cmd == CommandInterface::UserParams)
		{
			pComm->GetUserCommand(&usr_cmd);
			bNewMessage = true;
		}
		else
		{
			printf("Received unknown command: %d from UI!", cmd);
		}
		cmd_list->pop_front();
	}
	mtx.unlock();

	return bNewMessage;
}

bool Planner::CheckRobotState(list<CommandInterface::MESSAGE_TYPE>* cmd_list, CommHandler* pComm)
{
	bool bNewMessage = false;

	mtx.lock();
	while (!cmd_list->empty())
	{
		CommandInterface::MESSAGE_TYPE cmd = cmd_list->front();
		if (cmd == CommandInterface::StateInfo)
		{
			pComm->GetState(qpos, com, com_vel, left, right, &eOpState, &rcv_run_count);
			bNewMessage = true;
		}
		else
		{
			printf("Received unknown command: %d from robot!", cmd);
		}
		cmd_list->pop_front();
	}
	mtx.unlock();

	return bNewMessage;
}

#if LOGGING
void Planner::LogTrajectory() {
	fileTraj << targ_traj.numPoints << std::endl;
	for (int i = 0; i < targ_traj.numPoints; i++)
	{
		for (int j = 0; j < 4; j++)
			fileTraj << targ_traj.com_traj[i].com[j] << ",";
		for (int j = 0; j < 3; j++)
			fileTraj << targ_traj.com_traj[i].com_xdd[j] << ",";
		fileTraj << targ_traj.com_traj[i].com_xdd[3] << std::endl;
	}
}
#endif
