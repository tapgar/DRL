/*
 * OSC_RBDL.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: tapgar
 */

#include "OSC_RBDL.h"

using namespace std;
using namespace Eigen;


OSC_RBDL::OSC_RBDL(int* conIds, int* targIds) {
	m_nAssignedIndices = 0;
	contactSiteIds = conIds;
	targetSiteIds = targIds;
}

void OSC_RBDL::InitMatrices(DynamicModel* dyn) {

	//initialize weight matrix
	W = Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, XDD_TARGETS*DOF + QDD_TARGETS>::Zero();
	for (int i = 0; i < DOF; i++)
		W(i,i) = m_dWeight_COM;
	for (int i = DOF; i < XDD_TARGETS*DOF; i++)
		W(i,i) = m_dWeight_Stance;
	for (int i = XDD_TARGETS*DOF; i < XDD_TARGETS*DOF + QDD_TARGETS; i++)
		W(i,i) = m_dWeight_Rest;

	//initialize friction cone matrix
	V = Matrix<double, nC*DOF, nC*(2*(DOF-1)+1)>::Zero();
	for (int i = 0; i < nC; i++)
	{
		for (int j = 0; j < DOF-1; j++)
		{
			V(i*DOF+j, i*(2*(DOF-1)+1) + j*2) = -1.0;
			V(i*DOF+j, i*(2*(DOF-1)+1) + j*2 + 1) = 1.0;
		}
		V(i*DOF+DOF-1, i*(2*(DOF-1)+1) + 2*(DOF-1)) = 1.0; //normal force
	}

	//initialize inequality constraint matrix
	C = Matrix<double, nC*4*(DOF-1), nQ+nU+nC*(2*(DOF-1)+1)>::Zero();
	for (int i = 0; i < nC; i++)
	{
		for (int j = 0; j < DOF-1; j++)
		{
			int l = 0;
			for (int k = 0; k < 4; k++)
			{
				if (k % 2)
				{
					C(i*4*(DOF-1) + 4*j + k, nQ+nU+i*(2*(DOF-1)+1)+2*j+l) = 1.0;
					l++;
				}
				else
					C(i*4*(DOF-1) + 4*j + k, nQ+nU+i*(2*(DOF-1)+1)+2*j+l) = -1.0;
				C(i*4*(DOF-1) + 4*j + k, nQ+nU+(i+1)*(2*(DOF-1)+1)-1) = -mu;
			}
		}
	}

	//set up lb and ub
	lb = -Matrix<double, nQ+nU+nC*(2*(DOF-1)+1), 1>::Ones()*numeric_limits<double>::max();
	ub = Matrix<double, nQ+nU+nC*(2*(DOF-1)+1), 1>::Ones()*numeric_limits<double>::max();
	MatrixXd mlb = MatrixXd::Zero(nU,1);
	MatrixXd mub = MatrixXd::Zero(nU,1);
	GetMotorLimits(dyn, &mlb, &mub);
	lb.block<nU,1>(nQ,0) = mlb;
	ub.block<nU,1>(nQ,0) = mub;

	Matrix<double, nC*(2*(DOF-1) + 1), 1> flb = -Matrix<double, nC*(2*(DOF-1) + 1), 1>::Ones()*numeric_limits<double>::max();
	for (int i = 0; i < nC*(2*(DOF-1) + 1); i++)
		flb(i,0) = 0.0; //normal force always positive
	lb.block<nC*(2*(DOF-1) + 1),1>(nU+nQ,0) = flb;


	ciub = Matrix<double, nC*4*(DOF-1), 1>::Zero();
	cilb = -1.0*Matrix<double, nC*4*(DOF-1), 1>::Ones()*numeric_limits<double>::max();


	//Initialize everything to zeros
	M = MatrixXd::Zero(nQ,nQ);
	bias = VectorXd::Zero(nQ);
	Bt = MatrixXd::Zero(nQ, nU);
	Jc = MatrixXd::Zero(nC*DOF, nQ);
	A = MatrixXd::Zero(XDD_TARGETS*DOF + QDD_TARGETS, nQ);
	AdotQdot = VectorXd::Zero(XDD_TARGETS*DOF + QDD_TARGETS);
	Jeq = MatrixXd::Zero(nEQ, nQ);
	JeqdotQdot = VectorXd::Zero(nEQ);
	x = Matrix<double, nQ+nU+nC*(2*(DOF-1)+1), 1>::Zero();

	H = Matrix<double, nQ+nU+nC*(2*(DOF-1)+1), nQ+nU+nC*(2*(DOF-1)+1)>::Zero();
	gt = Matrix<double, 1, nQ+nU+nC*(2*(DOF-1)+1)>::Zero();
	CE = Matrix<double, nQ, nQ+nU+nC*(2*(DOF-1)+1)>::Zero();
	ce = Matrix<double, nQ, 1>::Zero();
}

void OSC_RBDL::GetMotorLimits(DynamicModel* dyn, MatrixXd* mlb, MatrixXd* mub)
{
	dyn->GetMotorLimits(mlb, mub);
}

void OSC_RBDL::UpdateMatricesAtState(DynamicModel* dyn)
{
	dyn->GetMassMatrix(&M);

	dyn->GetBiasForce(&bias);
	VectorXd passive = VectorXd::Zero(nQ);
	dyn->GetPassiveForce(&passive);
	bias -= passive;

	dyn->GetSelectorMatrix(&Bt);

	//Contact jacobians
	for (int i = 0; i < nC; i++)
	{
		MatrixXd jc = MatrixXd::Zero(DOF, nQ);
		dyn->GetSiteJacobian(&jc, contactSiteIds[i]);
		Jc.block<DOF,nQ>(i*DOF,0) = jc;
	}

	//target site jacobians
	for (int i = 0; i < XDD_TARGETS; i++)
	{
		MatrixXd jc = MatrixXd::Zero(DOF, nQ);
		VectorXd accel = VectorXd::Zero(DOF);
		dyn->GetSiteJacobian(&jc, targetSiteIds[i]);
		A.block<DOF,nQ>(i*DOF,0) = jc;
		dyn->GetSiteAccel(&accel, targetSiteIds[i]);
		AdotQdot.block<DOF,1>(i*DOF,0) = accel;
	}
	for (int i = 0; i < QDD_TARGETS; i++)
	{
		int idx = XDD_TARGETS*DOF + i;
		A(idx, qdd_targ_idx[i]) = 1.0;
	}

	//Constraint jacobians
	dyn->GetConstraintJacobian(&Jeq);
	dyn->GetConstraintAccel(&JeqdotQdot);

}

void OSC_RBDL::RunPTSC(DynamicModel* dyn, Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd, Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive, bool* bContact, Matrix<double, nU, 1>* u)
{

	UpdateMatricesAtState(dyn);

	Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, XDD_TARGETS*DOF + QDD_TARGETS> tempW = W;

	//adjust limits based on contact forces
	for (int i = 0; i < nC; i++)
	{
		if (bContact[i])
		{
			ub(nQ+nU+i*(2*(DOF-1)+1)+2*(DOF-1),0) = numeric_limits<double>::max();
			for (int j = 0; j < DOF; j++)
				tempW(DOF+(i/2)*DOF+j,DOF+(i/2)*DOF+j) = m_dWeight_Stance;
		}
		else
		{
			ub(nQ+nU+i*(2*(DOF-1)+1)+2*(DOF-1),0) = 0.0;
			for (int j = 0; j < DOF; j++)
				tempW(DOF+(i/2)*DOF+j,DOF+(i/2)*DOF+j) = m_dWeight_Swing;
		}
	}

	Matrix<double, nQ, nQ> I = Matrix<double, nQ, nQ>::Identity();
	Matrix<double, nQ, nQ> Hinv = M.inverse();
	Matrix<double, nEQ, nEQ> JHinvJ = pseudoinverse(Jeq*Hinv*Jeq.transpose());//.completeOrthogonalDecomposition().pseudoInverse();

	Nc = I - Jeq.transpose()*JHinvJ*Jeq*Hinv;
	gamma = Jeq.transpose()*JHinvJ*JeqdotQdot;

	for (int i = 0; i < DOF*XDD_TARGETS+QDD_TARGETS; i++)
		if (!bActive(i,0))
			tempW(i,i) = 0.0;

	//in case you added rows previously...
	CE.conservativeResize(nQ, nQ+nU+nC*(2*(DOF-1)+1));
	CE.block<nQ,nQ>(0,0) = M;
	CE.block<nQ,nU>(0,nQ) = -Nc*Bt;
	CE.block<nQ,nC*(2*(DOF-1)+1)>(0,nQ+nU) = -Nc*(Jc.transpose())*V;

	//in case you added rows previously...
	ce.conservativeResize(nQ,1);
	ce = -Nc*bias - gamma;

	H.block<nQ,nQ>(0,0) = 2.0*A.transpose()*tempW*A;

	int j = 0;
	for (int i = nQ+nU; i < H.cols(); i++)
	{
		if (j < 2*(DOF-1))
		{
			H(i,i) = m_dWeight_Fx;
		}
		else
		{
			H(i,i) = m_dWeight_Fz;
			j = -1;
		}
		j++;
	}

	gt.block<1,nQ>(0,0) = 2.0*AdotQdot.transpose()*tempW*A -2.0*xdd.transpose()*tempW*A;

	VectorXd xtemp;
	xtemp.resize(x.rows());
	for (int i = 0; i < x.rows(); i++)
		xtemp(i) = x(i,0);
	bool status = m_qp.solve(H.sparseView(), gt, CE.sparseView(), ce, C.sparseView(), cilb, ciub, lb, ub, xtemp);

//	(*u) = x.block<nU,1>(nQ,0);
	for (int i = 0; i < nU; i++)
		(*u)(i,0) = xtemp(nQ+i,0);

	if (!status)
		printf("QP SOLVE ERROR!!!\n");
}
