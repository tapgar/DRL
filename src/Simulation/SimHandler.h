/*
 * SimHandler.h
 *
 *  Created on: Dec 7, 2017
 *      Author: tapgar
 */

#ifndef SIMHANDLER_H_
#define SIMHANDLER_H_

#include "CommHandler.h"
#include "Cassie.h"
#include <sys/time.h>
#include <mutex>          // std::mutex

class SimHandler {
public:
	SimHandler();
	virtual ~SimHandler();

	void Run(std::list<CommandInterface::MESSAGE_TYPE>* cmd_list, CommHandler* pComm);

private:

	std::mutex mtx; //for accessing comm list

	Cassie* robot; //make general

	ROM_Policy_Struct targ_traj;

	double qpos[nX];
	double com[4];
	double comvel[4];
	double left[4];
	double right[4];
	CommandInterface::OP_STATE eOpState;

	uint32_t run_count;
	uint32_t rcv_run_count;
	double planTime_s;
	double stepTime_s;
	int nActiveIndex;

	struct timeval sim_time;

	void Update(bool bNew);
	void SendUpdate(CommHandler* pComm);
	bool CheckForNewCommands(std::list<CommandInterface::MESSAGE_TYPE>* cmd_list, CommHandler* pComm);

};

#endif /* SIMHANDLER_H_ */
