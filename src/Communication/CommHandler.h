#ifndef COMM_HANDLER_H
#define COMM_HANDLER_H

#include <stdio.h>
#include <thread>
#include <list>
#include <vector>
#include <sys/time.h>
#include "CommandInterface.h"
#include "udp_comms.h"
#include <mutex>          // std::mutex


using namespace std;

class CommHandler
{

public:

	CommHandler(bool bClient, unsigned int port);
	void ReceiveDataThread(list<CommandInterface::MESSAGE_TYPE>* cmd_list);
	void SendState(double* qpos, double* com, double* com_vel, double* left, double* right, CommandInterface::OP_STATE opmode, uint32_t run_count);
	void GetState(double* qpos, double* com, double* com_vel, double* left, double* right, CommandInterface::OP_STATE* opmode, uint32_t* run_count);
	void GetUserCommand(USER_Params* usr_cmd);
	void SendUserCommand(USER_Params* usr_cmd);
	void SendTrajectory(ROM_Policy_Struct* policy, int decimate);
	void GetTrajectory(ROM_Policy_Struct* policy);
	bool Connect();

private:

	udp_comms* comm;

	std::mutex mtx; //for accessing comm list


	struct timeval last_tx_time;
	static const long m_nTxTime_ms = 25;
	bool bReceivedData;

};
#endif
