/*
 * TestBenchInterface.cpp
 *
 *  Created on: Dec 24, 2017
 *      Author: tapgar
 */

#include "TestBenchInterface.h"

TestBenchInterface::TestBenchInterface() {
	// TODO Auto-generated constructor stub

}

TestBenchInterface::~TestBenchInterface() {
	// TODO Auto-generated destructor stub
}

void TestBenchInterface::Init(double* qpos_init, double* qvel_init) {
	cassie.LoadModel();
	cassie.setMeasuredStates(qpos_init, qvel_init);

	int contactIds[] = {0, 1};
	int targetIds[] = {0, 1};

	OSC_RBDL osc = OSC_RBDL(contactIds, targetIds);
	osc.AddQDDIdx(4);

	osc.InitMatrices(&cassie);
}
