/*
 * OSC.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: tapgar
 */

#include "OSC.h"

using namespace std;
using namespace Eigen;


OSC::OSC(int* conIds, int* targIds, int* bodyIds) {
	m_nAssignedIndices = 0;
	contactSiteIds = conIds;
	targetSiteIds = targIds;
	contactBodyIds = bodyIds;
}

void OSC::InitMatrices(const mjModel* m) {

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
	Matrix<double, nU, 1> mlb, mub;
	GetMotorLimits(m, &mlb, &mub);
	lb.block<nU,1>(nQ,0) = mlb;
	ub.block<nU,1>(nQ,0) = mub;
	//cout << "UB: " << endl << ub << endl;
	//cout << "LB: " << endl << lb << endl;
	Matrix<double, nC*(2*(DOF-1) + 1), 1> flb = -Matrix<double, nC*(2*(DOF-1) + 1), 1>::Ones()*numeric_limits<double>::max();
	for (int i = 0; i < nC*(2*(DOF-1) + 1); i++)
		flb(i,0) = 0.0; //normal force always positive
	lb.block<nC*(2*(DOF-1) + 1),1>(nU+nQ,0) = flb;

	//cout << "LB: " << endl << lb << endl;

	ciub = Matrix<double, nC*4*(DOF-1), 1>::Zero();
	cilb = -1.0*Matrix<double, nC*4*(DOF-1), 1>::Ones()*numeric_limits<double>::max();


	M_rotor = Matrix<double, nQ, nQ>::Zero();
//
//	for (int i = 0; i < nU; i++)
//	{
//		double gearN = m->actuator_gear[i*6];
////		//printf("ids: %d\t%d\n", m->actuator_trnid[i*2], m->actuator_trnid[i*2+1]);
//		int jnt_id = m->actuator_trnid[i*2];
//		M_rotor(jnt_id, jnt_id) = gearN*gearN*m->dof_armature[jnt_id];
//	}
//
//	//cout << "Rotor inertia matrix:" << endl << M_rotor << endl;

	//Initialize everything to zeros
	M = Matrix<double, nQ, nQ>::Zero();
	bias = Matrix<double, nQ, 1>::Zero();
	Bt = Matrix<double, nQ, nU>::Zero();
	Jc = Matrix<double, nC*DOF, nQ>::Zero();
	A = Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, nQ>::Zero();
	Adot = Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, nQ>::Zero();
	Jeq = Matrix<double, nEQ, nQ>::Zero();
	Jeqdot = Matrix<double, nEQ, nQ>::Zero();
	x = Matrix<double, nQ+nU+nC*(2*(DOF-1)+1), 1>::Zero();

	H = Matrix<double, nQ+nU+nC*(2*(DOF-1)+1), nQ+nU+nC*(2*(DOF-1)+1)>::Zero();
	gt = Matrix<double, 1, nQ+nU+nC*(2*(DOF-1)+1)>::Zero();
	CE = Matrix<double, nQ, nQ+nU+nC*(2*(DOF-1)+1)>::Zero();
	ce = Matrix<double, nQ, 1>::Zero();

	for (int i = 0; i < nQ; i++)
	{
		if (m->jnt_stiffness[m->dof_jntid[i]] > 1e-10)
			m_bSpringJoint[i] = true;
		else
			m_bSpringJoint[i] = false;
	}

}

void OSC::GetMotorLimits(const mjModel* m, Matrix<double, nU, 1>* mlb, Matrix<double, nU, 1>* mub)
{
	for (int i = 0; i < nU; i++)
	{
		if (!m->actuator_ctrllimited[i])
		{
			(*mlb)(i,0) = -1e20;
			(*mub)(i,0) = 1e20;
		}
		else {
			(*mlb)(i,0) = m->actuator_ctrlrange[i*2];
			(*mub)(i,0) = m->actuator_ctrlrange[i*2+1];
		}
	}
}

void OSC::UpdateMatricesAtState(const mjModel* m, const mjData* dmain)
{
	//NumericalJacobians(m, dmain);

	//quick data copy
    mjData* d = mj_makeData(m);
    d->time = dmain->time;
	mju_copy(d->qpos, dmain->qpos, m->nq);
	mju_copy(d->qvel, dmain->qvel, m->nv);
	mju_copy(d->qacc, dmain->qacc, m->nv);
	mju_copy(d->qacc_warmstart, dmain->qacc_warmstart, m->nv);
    mju_copy(d->qfrc_applied, dmain->qfrc_applied, m->nv);
    mju_copy(d->qfrc_actuator, dmain->qfrc_actuator, m->nv);
    mju_copy(d->qfrc_actuator, dmain->qfrc_passive, m->nv);
    mju_copy(d->xfrc_applied, dmain->xfrc_applied, 6*m->nbody);
	mju_copy(d->ctrl, dmain->ctrl, m->nu);

	mju_zero(d->ctrl, m->nu);

	for (int i = 0; i < 3; i++)
		mj_forward(m, d);

//	mj_copyData(d, m, dmain);		//copy data struct

	mjtNum mass[nQ*nQ];
	mju_zero(mass, nQ*nQ);
	mj_fullM(m, mass, d->qM);
	M = Map<Matrix<double,nQ,nQ,RowMajor>>(mass);
	M += M_rotor;

//	cout << "Mass:" << endl << M << endl;

	for (int i = 0; i < nQ; i++)
	{
		bias(i,0) = d->qfrc_bias[i] - d->qfrc_passive[i];
		//printf("%f\t%f\n", d->qfrc_bias[i], d->qfrc_passive[i]);
	}

//	cout << "Bias:" << endl << bias << endl;


	Bt = Map<Matrix<double, nQ, nU, ColMajor>>(d->actuator_moment);

//	cout << "Bt:" << endl << Bt << endl;
	//REMEMBER SNEAKY SHIT HERE TO GET DOF=2
	mjtNum jacp[nQ*3];
	mju_zero(jacp, nQ*3);
	for (int i = 0; i < nC; i++)
	{
		mj_jacSite(m, d, jacp, NULL, contactSiteIds[i]);
		for (int j = 0; j < DOF; j++)
			for (int k = 0; k < nQ; k++)
				Jc(i*DOF + j, k) = jacp[j*nQ + k];
	}

//	cout << "Jc:" << endl << Jc << endl;

	Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, nQ> Ap = Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, nQ>::Zero();
	Matrix<double, nEQ, nQ> Jeqp = Matrix<double, nEQ, nQ>::Zero();
	for (int iter = 0; iter < 2; iter++)
	{
		for (int i = 0; i < XDD_TARGETS; i++)
		{
			mj_jacSite(m, d, jacp, NULL, targetSiteIds[i]);
//			//printf("iter: %d\tsite:%d\n", iter, i);
//			//mju_printMat(jacp, 3, nQ);

			for (int j = 0; j < DOF; j++)
				for (int k = 0; k < nQ; k++)
				{
					if (iter == 0)
						A(i*DOF + j, k) = jacp[j*nQ + k];
					else
						Ap(i*DOF + j, k) = jacp[j*nQ + k];
				}
		}

		int row = 0;
		for (int i = 0; i < d->nefc; i++)
		{
			if (d->efc_type[i] != mjCNSTR_EQUALITY)
				continue;
			for (int j = 0; j < nQ; j++)
			{
				if (iter == 0)
					Jeq(row,j) = d->efc_J[i*nQ + j];
				else
					Jeqp(row,j) = d->efc_J[i*nQ + j];
			}
			row++;
		}
//		mju_printMat(d->qpos, 1, nQ);
		//take step forward in time to get jacobian time derivative
		mj_step(m, d);
		mj_forward(m, d);

//		mju_printMat(d->qpos, 1, nQ);
	}

	//add identity to QDD_TARG indices
	for (int i = 0; i < QDD_TARGETS; i++)
	{
		int idx = XDD_TARGETS*DOF + i;
		Ap(idx, qdd_targ_idx[i]) = 1.0;
		A(idx, qdd_targ_idx[i]) = 1.0;
	}

	Adot = (Ap - A)/m_dControlTime_s;
	Jeqdot = (Jeqp - Jeq)/m_dControlTime_s;

	////cout << "A:" << endl << A << endl;
	////cout << "Ap:" << endl << Ap << endl;
	////cout << "Adot:" << endl << Adot << endl;

	mj_deleteData(d);
}

void OSC::NumericalJacobians(const mjModel* m, const mjData* dmain)
{
	//quick data copy
	mjData* d = mj_makeData(m);
	d->time = dmain->time;
	mju_copy(d->qpos, dmain->qpos, m->nq);
	mju_copy(d->qvel, dmain->qvel, m->nv);
	mju_copy(d->qacc, dmain->qacc, m->nv);
	mju_copy(d->qacc_warmstart, dmain->qacc_warmstart, m->nv);
	mju_copy(d->qfrc_applied, dmain->qfrc_applied, m->nv);
	mju_copy(d->qfrc_actuator, dmain->qfrc_actuator, m->nv);
	mju_copy(d->qfrc_actuator, dmain->qfrc_passive, m->nv);
	mju_copy(d->xfrc_applied, dmain->xfrc_applied, 6*m->nbody);
	mju_copy(d->ctrl, dmain->ctrl, m->nu);

	for (int i = 0; i < 3; i++)
		mj_forward(m, d);

	mjtNum* output = d->qacc;
	mjtNum prev[nQ];
	mju_copy(prev, output, m->nv);

	mjtNum mass[nQ*nQ];
	mju_zero(mass, nQ*nQ);
	mj_fullM(m, mass, d->qM);

	mjtNum qfrc[nQ];
	mju_mulMatVec(qfrc, mass, d->qacc, nQ, nQ);
	mjtNum qfrc_orig[nQ];
	mju_copy(qfrc_orig, qfrc, nQ);

	Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, nQ> Anum = Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, nQ>::Zero();
	Matrix<double, 6, nQ> Jcnum = Matrix<double, 6, nQ>::Zero(); //contact jacobian

	int body_id = mj_name2id(m, mjOBJ_BODY, "left_toe");

	for (int i = 0; i < 6; i++)
	{
		d->xfrc_applied[6*body_id+i] += 0.001;
		mju_copy(d->qacc_warmstart, dmain->qacc_warmstart, m->nv);

		mj_forwardSkip(m, d, mjSTAGE_VEL, 1);

		mju_mulMatVec(qfrc, mass, d->qacc, nQ, nQ);

		for (int j = 0; j < m->nv; j++)
			Jcnum(i, j) = (qfrc[j] - qfrc_orig[j])/0.001;

		mju_copy(d->qacc, dmain->qacc, m->nv);

		d->xfrc_applied[6*body_id+i] = 0.0;
		mju_copy(qfrc_orig, qfrc, nQ);
	}

	////cout << "Jc" << endl << Jcnum << endl;

	mjtNum cacc[6];
	mj_objectAcceleration(m, d, mjOBJ_SITE, targetSiteIds[1], cacc, 0);
	mjtNum cacc_orig[6];
	mju_copy(cacc_orig, cacc, 6);

	for (int i = 0; i < nQ; i++)
	{
		d->qacc[i] += 0.001;
		d->qacc_warmstart[i] += 0.001;
//		for (int k = 0; k < 10; k++)
//			mj_forward(m, d);

	    mj_fwdAcceleration(m, d);
	    mj_sensorAcc(m, d);

		//mju_printMat(d->qacc, 1, nQ);

		mj_objectAcceleration(m, d, mjOBJ_SITE, targetSiteIds[1], cacc, 0);

		//mju_printMat(cacc, 1, 6);
		for (int j = 0; j < 6; j++)
			Anum(j,i) = (cacc[j] - cacc_orig[j])/0.001;

		d->qacc[i] -= 0.001;
		d->qacc_warmstart[i] -= 0.001;
	}

	////cout << "Anum: " << endl << Anum << endl;

	mj_deleteData(d);
}

void OSC::RunPTSC(const mjModel* m, const mjData* dmain, Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd, Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive, bool* bContact, Matrix<double, nU, 1>* u, Matrix<double, nC*DOF, 1>* f,  Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1>* xdd_e, Matrix<double, nQ, 1>* qdd_e, double* er)
{

	UpdateMatricesAtState(m, dmain);

	Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, XDD_TARGETS*DOF + QDD_TARGETS> tempW = W;

	//adjust limits based on contact forces
	for (int i = 0; i < nC; i++)
	{
		if (bContact[i])
		{
//			printf("%d in contact\n", i);
			ub(nQ+nU+i*(2*(DOF-1)+1)+2*(DOF-1),0) = numeric_limits<double>::max();
			for (int j = 0; j < DOF; j++)
				tempW(DOF+(i/2)*DOF+j,DOF+(i/2)*DOF+j) = m_dWeight_Stance;
		}
		else
		{
//			printf("%d not in contact\n", i);
			ub(nQ+nU+i*(2*(DOF-1)+1)+2*(DOF-1),0) = 0.0;
			for (int j = 0; j < DOF; j++)
				tempW(DOF+(i/2)*DOF+j,DOF+(i/2)*DOF+j) = m_dWeight_Swing;
//			for (int j = 2+12*(i/2); j < 2+12*((i/2)+1); j++) //pretty hacky... left/right leg
//			{
//				////printf("j: %d\n", j);
//				if (!m_bSpringJoint[j])
//					continue;
//				A((i+1)*DOF,j) = 0.0;
//				Adot((i+1)*DOF,j) = 0.0;
//				A((i+1)*DOF+1,j) = 0.0;
//				Adot((i+1)*DOF+1,j) = 0.0;
//			}
		}
	}

	Matrix<double, nQ, 1> qdd = Map<Matrix<double, nQ, 1>>(dmain->qacc);


	qd = Map<Matrix<double, nQ, 1>>(dmain->qvel);



	Matrix<double, nQ, nQ> I = Matrix<double, nQ, nQ>::Identity();
	////cout << "I: " << endl << I << endl;
	Matrix<double, nQ, nQ> Hinv = M.inverse();
	//cout << "JHJt: " << endl << (Jeq*Hinv*Jeq.transpose()) << endl;
	Matrix<double, nEQ, nEQ> JHinvJ = pseudoinverse(Jeq*Hinv*Jeq.transpose());//.completeOrthogonalDecomposition().pseudoInverse();
	//cout << "pinv: " << endl << JHinvJ << endl;

	Nc = I - Jeq.transpose()*JHinvJ*Jeq*Hinv;
	////cout << "Nc" << endl << Nc << endl;
	gamma = Jeq.transpose()*JHinvJ*Jeqdot*qd;
	////cout << "gamma" << endl << gamma << endl;

//	cout << "UB:" << endl << ub << endl;
//	cout << "LB:" << endl << lb << endl;

	//cout << "W:" << endl << tempW << endl;

//	cout << "V:" << endl << V << endl;

//	cout << "C:" << endl << C << endl;

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

//	for (int i = 0; i < nC; i++)
//	{
//		if (!bContact[i])
//			continue;
//		for (int j = i+1; j < nC; j++)
//		{
//			if (!bContact[j])
//				continue;
//
//			//same body make sure tangential forces are equal
//			if (contactBodyIds[j] == contactBodyIds[i])
//			{
//				CE.conservativeResize(CE.rows()+2,CE.cols());
//				CE.row(CE.rows()-2).setZero();
//				CE(CE.rows()-2, nQ+nU+i*(2*(DOF-1)+1)) = 1;
//				CE(CE.rows()-2, nQ+nU+j*(2*(DOF-1)+1)) = -1;
//				CE.row(CE.rows()-1).setZero();
//				CE(CE.rows()-1, nQ+nU+i*(2*(DOF-1)+1)+1) = 1;
//				CE(CE.rows()-1, nQ+nU+j*(2*(DOF-1)+1)+1) = -1;
//				ce.conservativeResize(ce.rows()+2,ce.cols());
//				ce(ce.rows()-2,0) = 0.0;
//				ce(ce.rows()-1,0) = 0.0;
////
////				CE.conservativeResize(CE.rows()+2,CE.cols());
////				CE.row(CE.rows()-2).setZero();
////				CE(CE.rows()-2, nQ+nU+i*(2*(DOF-1)+1)+2) = 1;
////				CE(CE.rows()-2, nQ+nU+j*(2*(DOF-1)+1)+2) = -1;
////				CE.row(CE.rows()-1).setZero();
////				CE(CE.rows()-1, nQ+nU+i*(2*(DOF-1)+1)+3) = 1;
////				CE(CE.rows()-1, nQ+nU+j*(2*(DOF-1)+1)+3) = -1;
////				ce.conservativeResize(ce.rows()+2,ce.cols());
////				ce(ce.rows()-2,0) = 0.0;
////				ce(ce.rows()-1,0) = 0.0;
////				printf("Adding rows...\n");
//			}
//		}
//	}

//	cout << "Ax = b" << endl << CE << endl;
//	cout << "Ax = b" << endl << ce << endl;

//	for (int i = 0; i < nC; i++)
//		printf("%d\n", contactBodyIds[i]);
	//cout << "gamma" << endl << gamma << endl;
	//cout << "bias" << endl << Nc*bias << endl;

	H.block<nQ,nQ>(0,0) = 2.0*A.transpose()*tempW*A;
	for (int i = nQ; i < nQ+nU; i++)
		H(i,i) = m_dWeight_Tau*m->actuator_gear[(i-nQ)*6];
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
//	cout << "H:" << endl << H << endl;

	gt.block<1,nQ>(0,0) = 2.0*(Adot*qd).transpose()*tempW*A -2.0*xdd.transpose()*tempW*A;

	////cout << "gt:" << endl << gt << endl;

	VectorXd xtemp;
	xtemp.resize(x.rows());
	for (int i = 0; i < x.rows(); i++)
		xtemp(i) = x(i,0);
	bool status = m_qp.solve(H.sparseView(), gt, CE.sparseView(), ce, C.sparseView(), cilb, ciub, lb, ub, xtemp);

//	(*u) = x.block<nU,1>(nQ,0);
	for (int i = 0; i < nU; i++)
		(*u)(i,0) = xtemp(nQ+i,0);

	Matrix<double, nC*(2*(DOF-1)+1), 1> lambda;
	for (int i = 0; i < nC*(2*(DOF-1)+1); i++)
		lambda(i,0) = xtemp(nQ+nU+i,0);
	*f = V*lambda;

	for (int i = 0; i < nQ; i++)
		(*qdd_e)(i,0) = xtemp(i,0);

	*xdd_e = A*(*qdd_e) + Adot*qd;

//	cout << "A" << endl << A << endl;
//	cout << "Aqdd" << endl << A*(*qdd_e) << endl;
//	cout << "Adotqdot" << endl << Adot*qd << endl;
//
//	cout << "xdd" << endl <<  xdd << endl;



	for (int i = 0; i < x.rows(); i++)
		x(i,0) = xtemp(i);

	//for the purposes of calculating residual error

	Matrix<double, 1, 1> qp_er = 0.5*(x.transpose()*H*x) + gt*x + (Adot*qd).transpose()*tempW*(Adot*qd - xdd) + xdd.transpose()*tempW*(xdd - Adot*qd);
	*er = qp_er(0,0);

//	printf("QP error: %f\n", *er);

//	cout << xtemp.transpose() << endl;

	//cout << "Jeq" << endl << Jeq << endl;

//	//cout << qdd_e.transpose() << endl;

	//cout << qd.transpose() << endl;
//
//	printf("mujoco constraint forces: \n");
//	for (int i = 0; i < dmain->nefc; i++)
//	{
////		if (dmain->efc_type[i] != mjCNSTR_EQUALITY)
////			continue;
//		printf("%f\n",dmain->efc_force[i]);
//		//mju_printMat(&(dmain->efc_J[i*nQ]), 1, nQ);
//	}
//	mju_printMat(dmain->qfrc_constraint, 1, nQ);
////
////	//cout << "constraint vels: " << endl << Jeq*qd << endl;
////
////	//printf("mujoco constraint vels: \n");
////	for (int i = 0; i < dmain->nefc; i++)
////	{
////		if (dmain->efc_type[i] != mjCNSTR_EQUALITY)
////			continue;
////		//printf("%f\n",dmain->efc_vel[i]);
////	}
//
//	//cout << "actuator torque:" << endl << Bt*(*u) << endl;
//
//	Matrix<double, nEQ, 1> f_constraint;
//	f_constraint = JHinvJ*(Jeq*Hinv*bias - Jeqdot*qd - Jeq*Hinv*Bt*(*u));
//
//	cout << "Constraint force:" << endl << f_constraint << endl;
//
//	cout << "JHinvJ" << endl << JHinvJ << endl;
//
//	cout << "Jeq" << endl << Jeq << endl;
//
//	cout << "JeqHinvbias" << endl << Jeq*Hinv*bias << endl;
//
//	cout << "Jeqdotqd" << endl << Jeqdot*qd << endl;
//	cout << "JeqHinvBtu" << endl << Jeq*Hinv*Bt*(*u) << endl;


	if (!status)
		printf("QP SOLVE ERROR!!!\n");


}
