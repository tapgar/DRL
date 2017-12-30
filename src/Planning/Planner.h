/*
 * Planner.h
 *
 *  Created on: Dec 8, 2017
 *      Author: tapgar
 */

#ifndef PLANNER_H_
#define PLANNER_H_

#include "MPC.h"
#include "CommHandler.h"
#include "CommandInterface.h"
#include <fstream>

#define LOGGING false

class Planner {
public:
	Planner();
	virtual ~Planner();

	void Run(list<CommandInterface::MESSAGE_TYPE>* robot_msg, CommHandler* pRobotComm, list<CommandInterface::MESSAGE_TYPE>* ui_cmd_list, CommHandler* pUIComm);

private:

	std::mutex mtx;

	double qpos[nX];
	double com[4];
	double com_vel[4];
	double left[4];
	double right[4];
	uint32_t rcv_run_count;
	CommandInterface::OP_STATE eOpState;
	USER_Params usr_cmd;

	ROM_Policy_Struct targ_traj;

	Ipopt::MPC mpc;

	MPC_OPTIONS* opt;

	bool CheckForNewCommands(list<CommandInterface::MESSAGE_TYPE>* cmd_list, CommHandler* pComm);
	bool CheckRobotState(list<CommandInterface::MESSAGE_TYPE>* cmd_list, CommHandler* pComm);

	bool CompareOpPhase(CommandInterface::OP_STATE op_state, PHASE_STATE con_state);
	bool UpdateContactSchedule();

	bool Update();
	void SendUIUpdate(CommHandler* pComm);
	void SendRobotUpdate(CommHandler* pComm);

#if LOGGING
	void LogTrajectory();
	ofstream fileTraj;
#endif

};

#endif /* PLANNER_H_ */
