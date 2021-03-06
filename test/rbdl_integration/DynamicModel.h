/*
 * DynamicModel.h
 *
 *  Created on: Dec 20, 2017
 *      Author: tapgar
 */

#ifndef DYNAMICMODEL_H_
#define DYNAMICMODEL_H_

#include <rbdl/rbdl.h>
#include "xml_parser.h"
#include <Eigen/Dense>
#include "SharedRobotDefinitions.h"

struct RobotState {
	RigidBodyDynamics::Math::VectorNd qpos;
	RigidBodyDynamics::Math::VectorNd qvel;
};

struct Site {
	RigidBodyDynamics::Math::Vector3d pos;
	unsigned int body_id;
	std::string name;
};

struct Constraint {
	RigidBodyDynamics::Math::Vector3d posA;
	unsigned int bodyA_id;

	RigidBodyDynamics::Math::Vector3d posB;
	unsigned int bodyB_id;
};

class DynamicModel {
public:
	DynamicModel();
	virtual ~DynamicModel();

	void LoadModel();

	void setState(double* qpos, double* qvel);

	void GetMassMatrix(RigidBodyDynamics::Math::MatrixNd* mat);
	void GetConstraintJacobian(RigidBodyDynamics::Math::MatrixNd* mat);
	void GetConstraintAccel(RigidBodyDynamics::Math::VectorNd* accel);

	void GetBiasForce(RigidBodyDynamics::Math::VectorNd* qfrc_bias);
	void GetPassiveForce(RigidBodyDynamics::Math::VectorNd* qfrc_passive);

	void GetSiteJacobian(RigidBodyDynamics::Math::MatrixNd* mat, int siteId);
	void GetSiteAccel(RigidBodyDynamics::Math::VectorNd* accel, int siteId);

	void GetSelectorMatrix(RigidBodyDynamics::Math::MatrixNd* mat);

	void GetMotorLimits(RigidBodyDynamics::Math::MatrixNd* mlb, RigidBodyDynamics::Math::MatrixNd* mub);

	void GetTargetPoints(RigidBodyDynamics::Math::VectorNd* x, RigidBodyDynamics::Math::VectorNd* xd);

private:

	RigidBodyDynamics::Model m;
	RobotState m_State;

	RigidBodyDynamics::Math::MatrixNd rotorInertia;
	RigidBodyDynamics::Math::MatrixNd selectorMatrix;
	RigidBodyDynamics::Math::VectorNd jointDamping;

	RigidBodyDynamics::Math::MatrixNd motorLimits;

	std::vector<Site> m_Sites;
	std::vector<Constraint> m_Constraints;

};

#endif /* DYNAMICMODEL_H_ */
