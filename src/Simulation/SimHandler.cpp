/*
 * SimHandler.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: tapgar
 */

#include "SimHandler.h"

SimHandler::SimHandler() {
	// TODO Auto-generated constructor stub
	robot = new Cassie();
	eOpState = CommandInterface::Idle;
	planTime_s = 0.0;
	stepTime_s = 0.0;
	nActiveIndex = 0;
	run_count = 0;
	rcv_run_count = 0;

	targ_traj.dt_c = 0.0;
	ROM_TrajPt_Struct null;
	for (int i = 0; i < MAX_TRAJ_PTS; i++)
		targ_traj.com_traj.push_back(null);
	ContactInfo_Struct null_con;
	for (int i = 0; i < MAX_CON_SWITCH; i++)
		targ_traj.con_sched.push_back(null_con);
}

SimHandler::~SimHandler() {
	// TODO Auto-generated destructor stub
}

void SimHandler::Run(list<CommandInterface::MESSAGE_TYPE>* cmd_list, CommHandler* pComm)
{
	bool bNew = CheckForNewCommands(cmd_list, pComm);

	Update(bNew);

	SendUpdate(pComm);
}

bool SimHandler::CheckForNewCommands(list<CommandInterface::MESSAGE_TYPE>* cmd_list, CommHandler* pComm)
{
	bool bNew = false;
	mtx.lock(); //make sure CommHandler isn't adding new commands

	while (!cmd_list->empty())
	{
		CommandInterface::MESSAGE_TYPE cmd = cmd_list->front();
		if (cmd == CommandInterface::NewTrajectory)
		{
			pComm->GetTrajectory(&targ_traj);
			bNew = true;
		}
		else
		{
			printf("Received unknown command: %d from robot!", cmd);
		}
		cmd_list->pop_front();
	}

	mtx.unlock();

	return bNew;
}

void SimHandler::Update(bool bNewTraj) {

	static double swing_time = 0.0;
	static double foot_apex_targ[3];

	if (bNewTraj)
	{
		planTime_s = double(run_count - targ_traj.run_count)*0.0005;
		nActiveIndex = 0;
	}

	if ((eOpState != CommandInterface::Idle && eOpState != CommandInterface::Standing) || bNewTraj)
	{
		for (int i = 0; i < targ_traj.numContactSwitch; i++)
		{
			if (planTime_s >= targ_traj.con_sched[i].T_start && planTime_s < targ_traj.con_sched[i].T_end)
			{
				if (targ_traj.con_sched[i].con_state == Double)
					eOpState = CommandInterface::Walking_DS;
				else if (targ_traj.con_sched[i].con_state == SS_Left)
				{
					eOpState = CommandInterface::Walking_SS_Left;
					if (i > nActiveIndex)
					{
//						printf("right apex target:\n");
						for (int j = 0; j < 3; j++)
						{
							foot_apex_targ[j] = (right[j] + targ_traj.con_sched[i].right[j])/2.0;
//							printf("%f\t%f\t%f\n", right[j], targ_traj.con_sched[i].right[j], foot_apex_targ[j]);
						}
						swing_time = targ_traj.con_sched[i].T_end - targ_traj.con_sched[i].T_start;
					}
				}
				else if (targ_traj.con_sched[i].con_state == SS_Right)
				{
					eOpState = CommandInterface::Walking_SS_Right;
					if (i > nActiveIndex)
					{
//						printf("left apex target:\n");
						for (int j = 0; j < 3; j++)
						{
							foot_apex_targ[j] = (left[j] + targ_traj.con_sched[i].left[j])/2.0;
//							printf("%f\t%f\t%f\n", left[j], targ_traj.con_sched[i].left[j], foot_apex_targ[j]);
						}
						swing_time = targ_traj.con_sched[i].T_end - targ_traj.con_sched[i].T_start;
					}
				}
				if (i > nActiveIndex)
					stepTime_s = 0.0;
				nActiveIndex = i;
				break;
			}
		}
	}

	foot_apex_targ[2] = targ_traj.step_height;//step height

	robot->Update(&targ_traj, eOpState, nActiveIndex, planTime_s, stepTime_s, foot_apex_targ, swing_time);
	planTime_s += 0.0005;
	stepTime_s += 0.0005;
	run_count++;
}

void SimHandler::SendUpdate(CommHandler* pComm)
{
	robot->GetState(qpos);
	robot->GetCOMFeet(com, comvel, left, right);
	pComm->SendState(qpos, com, comvel, left, right, eOpState, run_count);
}
