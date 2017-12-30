/*
 * RobotDefinitions.h
 *
 *  Created on: Dec 7, 2017
 *      Author: tapgar
 */

#ifndef ROBOTDEFINITIONS_H_
#define ROBOTDEFINITIONS_H_

#define nQ 5 //# joints
#define nX nQ //pos=vel+1 (quaternion)
#define nU 3 //# actuated joints
#define nC 2 //# contact points
#define nEQ 3 //#of equality constraints 4*3

#define XDD_TARGETS 2 //# of cartesian target accelerations
#define QDD_TARGETS 1 //# of joint target accels

#define DOF 3 // 2D vs 3D

static const double m_dControlTime_s = 0.0005; //how often the controller runs

static const double m_dNominalZTarget_m = 0.95;




#endif /* ROBOTDEFINITIONS_H_ */
