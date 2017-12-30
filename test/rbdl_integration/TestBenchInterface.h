/*
 * TestBenchInterface.h
 *
 *  Created on: Dec 24, 2017
 *      Author: tapgar
 */

#ifndef TESTBENCHINTERFACE_H_
#define TESTBENCHINTERFACE_H_

#include "DynamicModel.h"
#include "OSC_RBDL.h"

class TestBenchInterface {
public:
	TestBenchInterface();
	virtual ~TestBenchInterface();

	void Init(double* qpos_init, double* qvel_init);

	void Run(double* q, double* qd, double* xdd, bool* bInContact);

private:
	DynamicModel cassie;
	OSC_RBDL osc;
};

#endif /* TESTBENCHINTERFACE_H_ */
