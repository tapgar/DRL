
#include "CommHandler.h"
#include <sys/time.h>

using namespace std;

CommHandler::CommHandler(bool bClient, unsigned int port) {
	comm = new udp_comms(bClient, port);
	gettimeofday(&last_tx_time, NULL);
	bReceivedData = false;
}

bool CommHandler::Connect() {
	return comm->conn();
}

void CommHandler::ReceiveDataThread(list<CommandInterface::MESSAGE_TYPE>* cmd_list)
{
	while (1)
	{
		CommandInterface::MESSAGE_TYPE next_cmd;
		if (comm->receive_command(&next_cmd))
		{
			mtx.lock();
			cmd_list->push_back(next_cmd);
			mtx.unlock();
			bReceivedData = true;
		}
	}
}

void CommHandler::SendState(double* qpos, double* com, double* com_vel, double* left, double* right, CommandInterface::OP_STATE opmode, uint32_t run_count) {
	CommandInterface::StateInfo_Struct state;
	state.op_mode = (uint8_t)opmode;

	for (int i = 0; i < nX; i++)
		state.x_mm[i] = int(1e6*qpos[i]);

	state.run_count = run_count;

	for (int i = 0; i < 4; i++)
	{
		state.com[i] = int(1e6*com[i]);
		state.com_vel[i] = int(1e6*com_vel[i]);
		state.left[i] = int(1e6*left[i]);
		state.right[i] = int(1e6*right[i]);
	}

	struct timeval cur_tx_time;
	gettimeofday(&cur_tx_time, NULL);

	long mtime = ((cur_tx_time.tv_sec - last_tx_time.tv_sec)*1000.0 + (cur_tx_time.tv_usec - last_tx_time.tv_usec)/1000.0);

	if (mtime > m_nTxTime_ms)
	{
//		printf("sending state...\n");
		comm->send_state_info(state);
		gettimeofday(&last_tx_time, NULL);
	}
}

void CommHandler::GetState(double* qpos, double* com, double* com_vel, double* left, double* right, CommandInterface::OP_STATE* opmode, uint32_t* run_count) {
	CommandInterface::StateInfo_Struct state;
	comm->get_state(&state);
	*opmode = (CommandInterface::OP_STATE)state.op_mode;
	*run_count = state.run_count;

	for (int i = 0; i < nX; i++)
		qpos[i] = double(state.x_mm[i])/1e6;
	for (int i = 0; i < 4; i++)
	{
		com[i] = double(state.com[i])/1e6;
		com_vel[i] = double(state.com_vel[i])/1e6;
		left[i] = double(state.left[i])/1e6;
		right[i] = double(state.right[i])/1e6;
	}
}

void CommHandler::GetTrajectory(ROM_Policy_Struct* policy)
{
	//get pointer to commaction struct
	CommAction_Struct* pCA = &(comm->newDataBuff);
	policy->dt_c = double(pCA->trajInfo.dt_c)/1e6;
	policy->step_height = double(pCA->trajInfo.step_height)/1e6;
	policy->numContactSwitch = pCA->trajInfo.numContactSwitch;
	policy->numPoints = pCA->trajInfo.numPoints;
	policy->run_count = pCA->trajInfo.run_count;

	for (unsigned int i = 0; i < policy->numContactSwitch; i++)
	{
		policy->con_sched[i].con_state = pCA->conSched[i].con_state;
		policy->con_sched[i].T_start = double(pCA->conSched[i].T_start)/1e6;
		policy->con_sched[i].T_end = double(pCA->conSched[i].T_end)/1e6;
		for (unsigned int j = 0; j < 4; j++)
		{
			policy->con_sched[i].left[j] = double(pCA->conSched[i].left[j])/1e6;
			policy->con_sched[i].right[j] = double(pCA->conSched[i].right[j])/1e6;
		}
	}

	for (unsigned int i = 0; i < policy->numPoints; i++)
		for (unsigned int j = 0; j < 4; j++)
		{
			policy->com_traj[i].com[j] = double(pCA->posTraj[i].com[j])/1e6;
			policy->com_traj[i].com_xdd[j] = double(pCA->posTraj[i].com_xdd[j])/1e6;
		}
}

void CommHandler::SendTrajectory(ROM_Policy_Struct* policy, int decimate)
{
	//get pointer to commaction struct
	CommAction_Struct* pCA = &(comm->newDataBuff);
	pCA->trajInfo.dt_c = int32_t(1e6*policy->dt_c)*decimate;
	pCA->trajInfo.step_height = uint32_t(1e6*policy->step_height);
	pCA->trajInfo.numContactSwitch = policy->numContactSwitch;
	pCA->trajInfo.numPoints = min(MAX_TRAJ_PTS, policy->numPoints/decimate);
	pCA->trajInfo.run_count = policy->run_count;

	for (int i = 0; i < policy->numContactSwitch; i++)
	{
		pCA->conSched[i].con_state = policy->con_sched[i].con_state;
		pCA->conSched[i].T_start = int32_t(policy->con_sched[i].T_start*1e6);
		pCA->conSched[i].T_end = int32_t(policy->con_sched[i].T_end*1e6);
		for (int j = 0; j < 4; j++)
		{
			pCA->conSched[i].left[j] = int32_t(policy->con_sched[i].left[j]*1e6);
			pCA->conSched[i].right[j] = int32_t(policy->con_sched[i].right[j]*1e6);
		}
	}

	for (int i = 0; i < pCA->trajInfo.numPoints*decimate; i+=decimate)
		for (int j = 0; j < 4; j++)
		{
			pCA->posTraj[i/decimate].com[j] = int32_t(policy->com_traj[i].com[j]*1e6);
			pCA->posTraj[i/decimate].com_xdd[j] = int32_t(policy->com_traj[i].com_xdd[j]*1e6);
		}

	comm->send_trajectory();
}

void CommHandler::GetUserCommand(USER_Params* usr_cmd)
{
	CommandInterface::USER_Params usr_params = comm->newDataBuff.usr_cmd;
	usr_cmd->ds_perc = double(usr_params.ds_perc)/1e6;
	usr_cmd->step_time = double(usr_params.step_time_us)/1e6;
	usr_cmd->step_height = double(usr_params.step_height)/1e6;
	usr_cmd->xT.x_m = double(usr_params.dxT[0])/1e6;
	usr_cmd->xT.y_m = double(usr_params.dxT[1])/1e6;
	usr_cmd->xT.z_m = double(usr_params.dxT[2])/1e6;
	usr_cmd->heading = double(usr_params.dxT[3])/1e6;

}

void CommHandler::SendUserCommand(USER_Params* usr_cmd)
{
	CommandInterface::USER_Params usr_params;
	usr_params.ds_perc = uint32_t(1e6*usr_cmd->ds_perc);
	usr_params.step_time_us = uint32_t(1e6*usr_cmd->step_time);
	usr_params.step_height = uint32_t(1e6*usr_cmd->step_height);
	usr_params.dxT[0] = int32_t(1e6*usr_cmd->xT.x_m);
	usr_params.dxT[1] = int32_t(1e6*usr_cmd->xT.y_m);
	usr_params.dxT[2] = int32_t(1e6*usr_cmd->xT.z_m);
	usr_params.dxT[3] = int32_t(1e6*usr_cmd->heading);
	comm->send_usr_cmd(usr_params);
}
