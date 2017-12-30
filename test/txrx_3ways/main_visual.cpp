#include <stdio.h>
#include <iostream>
#include <thread>

#include "CommHandler.h"
#include "CommandInterface.h"
#include <mutex>          // std::mutex

#include "mujoco.h"
#include "Visualizer.h"

using namespace std;

int main(int argc,char* argv[]) {

	bool bSaveVid = false;
	if (argc > 1)
	{
		if (strcmp(argv[1],"save") == 0)
			bSaveVid = true;
		else
		{
			printf("unknown option: %s\nexiting\n", argv[1]);
			return -1;
		}
	}

	CommHandler* comms = new CommHandler(false, 8880);
	if (!comms->Connect())
	{
		printf("Failed to connect... returning\n");
		return -1;
	}

	list<CommandInterface::MESSAGE_TYPE>* cmd_list = new list<CommandInterface::MESSAGE_TYPE>;
	thread comm_thread(&CommHandler::ReceiveDataThread, comms, cmd_list);

	mutex mtx;
	double qpos[nX];

	mj_activate("/home/tapgar/.mujoco/mjkey.txt");
	char error[1000] = "Could not load binary model";
	mjModel* mj_Model = mj_loadXML("/home/tapgar/cuda-workspace/DRL/models/cassie/cassie3d_stiff.xml", 0, error, 1000);
	if (!mj_Model) {
		mju_error_s("Load model error: %s", error);
		return -1;
	}
	// Initialize mjData
	mjData* mj_Data = mj_makeData(mj_Model);

	Visualizer* vis = new Visualizer(mj_Model, bSaveVid, "remote_cassie");

	ROM_Policy_Struct targ_traj;
	targ_traj.dt_c = 0.0;
	ROM_TrajPt_Struct null;
	for (int i = 0; i < MAX_TRAJ_PTS; i++)
		targ_traj.com_traj.push_back(null);
	ContactInfo_Struct null_con;
	for (int i = 0; i < MAX_CON_SWITCH; i++)
		targ_traj.con_sched.push_back(null_con);

	double com[4], comvel[4], left[4], right[4];
	CommandInterface::OP_STATE eOpState;
	uint32_t run_count;
	USER_Params usr_opt;

	while (true) {
		bool bNew = false;
		mtx.lock();
		while (!cmd_list->empty())
		{
			CommandInterface::MESSAGE_TYPE cmd = cmd_list->front();
			if (cmd == CommandInterface::StateInfo)
			{
				comms->GetState(mj_Data->qpos, com, comvel, left, right, &eOpState, &run_count);
				bNew = true;
			}
			else if (cmd == CommandInterface::NewTrajectory)
			{
				comms->GetTrajectory(&targ_traj);
				vis->SetMPCPlan(&targ_traj);
				printf("Trajectory: %d points\t%d contact\t%f dt\n", targ_traj.numPoints, targ_traj.numContactSwitch, targ_traj.dt_c);
			}
			cmd_list->pop_front();

		}
		mtx.unlock();
		if (bNew)
		{
			mj_forward(mj_Model, mj_Data);
			vis->Draw(mj_Data);
			usr_opt.bNewPlan = false;
			vis->UpdateUserParams(&usr_opt);
			if (usr_opt.bNewPlan)
				comms->SendUserCommand(&usr_opt);
		}
	}

}
