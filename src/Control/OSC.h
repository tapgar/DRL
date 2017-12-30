/*
 * OSC.h
 *
 *  Created on: Nov 8, 2017
 *      Author: tapgar
 */

#ifndef OSC_H_
#define OSC_H_

#include "mujoco.h"
#include "OOPInterface.h" //required for QP
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/QR>
#include "Common_Structs.h"
#include "HelperFunctions.h"
#include "SharedRobotDefinitions.h"


class OSC {

public:
	OSC(int* conIds, int* targIds, int* bodyIds);

	void RunPTSC(const mjModel* m, const mjData* dmain, Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd, Eigen::Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive, bool* bContact, Eigen::Matrix<double, nU, 1>* u, Eigen::Matrix<double, nC*DOF, 1>* f,  Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1>* xdd_e, Eigen::Matrix<double, nQ, 1>* qdd_e, double* residual);

	int AddQDDIdx(int idx) {

		if (m_nAssignedIndices == QDD_TARGETS || idx < 0 || idx >= nQ)
			return 1;
		qdd_targ_idx[m_nAssignedIndices] = idx;
		m_nAssignedIndices++;
		return 0;
	}

	void InitMatrices(const mjModel* m);


private:

	OOPInterface m_qp;

	void GetMotorLimits(const mjModel* m, Eigen::Matrix<double, nU, 1>* mlb, Eigen::Matrix<double, nU, 1>* mub);

	void UpdateMatricesAtState(const mjModel* m, const mjData* dmain);
	void NumericalJacobians(const mjModel* m, const mjData* dmain);

	Eigen::Matrix<double, nQ, nQ> M; //Mass matrix
	Eigen::Matrix<double, nQ, nQ> M_rotor; //Rotor inertia matrix
	Eigen::Matrix<double, nQ, 1> bias; //coriolis, grav, spring
	Eigen::Matrix<double, nQ, nU> Bt; //B transpose
	Eigen::Matrix<double, nC*DOF, nQ> Jc; //contact jacobian
	Eigen::Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, nQ> A; //jacobian relating target
	Eigen::Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, nQ> Adot; //time deriv above
	Eigen::Matrix<double, nEQ, nQ> Jeq; //constrian jacobian
	Eigen::Matrix<double, nEQ, nQ> Jeqdot;
	Eigen::Matrix<double, nQ, 1> qd; //joint vels

	Eigen::Matrix<double, nQ, nQ> Nc; //constraint projector
	Eigen::Matrix<double, nQ, 1> gamma; //

	Eigen::Matrix<double, nC*DOF, nC*(2*(DOF-1)+1)> V; //friction cone

	Eigen::Matrix<double, nQ+nU+nC*(2*(DOF-1)+1), 1> x;//open vars

	Eigen::Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, XDD_TARGETS*DOF + QDD_TARGETS> W; //QP weighting matrix

	//QP matrices
	// z = 0.5*x'Hx + g'x;
	Eigen::Matrix<double, nQ+nU+nC*(2*(DOF-1)+1), nQ+nU+nC*(2*(DOF-1)+1)> H;//hessian
	Eigen::Matrix<double, 1, nQ+nU+nC*(2*(DOF-1)+1)> gt; //jacobian
	//Equality constraints
	//Ax = b
//	Eigen::Matrix<double, nQ, nQ+nU+nC*(2*(DOF-1)+1)> CE;
//	Eigen::Matrix<double, nQ, 1> ce;
	Eigen::MatrixXd CE;
	Eigen::MatrixXd ce;
	//Inequality constraints
	//d <= Cx <= f
	Eigen::Matrix<double, nC*4*(DOF-1), nQ+nU+nC*(2*(DOF-1)+1)> C;
	Eigen::Matrix<double, nC*4*(DOF-1), 1> cilb;
	Eigen::Matrix<double, nC*4*(DOF-1), 1> ciub;
	//Upper/Lower bounds
	Eigen::Matrix<double, nQ+nU+nC*(2*(DOF-1)+1), 1> ub;
	Eigen::Matrix<double, nQ+nU+nC*(2*(DOF-1)+1), 1> lb;

	int qdd_targ_idx[QDD_TARGETS];
	int m_nAssignedIndices;

	bool m_bSpringJoint[nQ];

	static constexpr double m_dWeight_Stance = 1e1;
	static constexpr double m_dWeight_Swing = 1e0;//0.2;
	static constexpr double m_dWeight_COM = 5.0;
	static constexpr double m_dWeight_Rest = 1e-1;//0.000002;
	static constexpr double m_dWeight_Tau = 1e-6;//0.000002;
	static constexpr double m_dWeight_Fx = 0.0;
	static constexpr double m_dWeight_Fz = 1e-4;

	int* contactSiteIds;
	int* targetSiteIds;
	int* contactBodyIds;

//	void pinv( MatrixType& pinvmat) const
//	{
//		eigen_assert(m_isInitialized && "SVD is not initialized.");
//		double  pinvtoler=1.e-6; // choose your tolerance wisely!
//		SingularValuesType singularValues_inv=m_singularValues;
//		for ( long i=0; i<m_workMatrix.cols(); ++i) {
//			if ( m_singularValues(i) > pinvtoler )
//				singularValues_inv(i)=1.0/m_singularValues(i);
//			else singularValues_inv(i)=0;
//		}
//		pinvmat= (m_matrixV*singularValues_inv.asDiagonal()*m_matrixU.transpose());
//	}

};


#endif /* OSC_H_ */
