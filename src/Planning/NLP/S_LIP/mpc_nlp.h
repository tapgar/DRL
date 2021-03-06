// Copyright (C) 2005, 2007 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: hs071_nlp.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-09

#ifndef __MPC_NLP_HPP__
#define __MPC_NLP_HPP__

#include "IpTNLP.hpp"
#include <string.h>
#include "math.h"
#include "Common_Structs.h"
#include "CommandInterface.h"
#include <vector>

#define MAX_IPOPT_VARS 50000

#pragma pack(push, 1)
typedef struct {
	Ipopt::Number c0[8]; //com pos (3 translation + 1 rotation)
	Ipopt::Number pfL[3]; //left foot pos
	Ipopt::Number pfR[3]; //right foot pos
	Ipopt::Number lam0[4];
	Ipopt::Number lamT[4];
	Ipopt::Number r0;
	Ipopt::Number rT;
	Ipopt::Number add;
}PHASE_Params;
#pragma pack(pop)

typedef struct {
	PHASE_STATE eType;
	Ipopt::Number T;
} PHASE_Info;

class MPC_OPTIONS {

public:
	MPC_OPTIONS() {
		N = 0;
		M = 0;
	}

	void setFoot(Ipopt::Number* x_vec, int idx, Ipopt::Number px, Ipopt::Number py, Ipopt::Number a)
	{
		x_vec[8+idx*3] = px;
		x_vec[9+idx*3] = py;
		x_vec[10+idx*3] = a;
	}

	void calc_cop(const PHASE_Params* p, Ipopt::Number t, Ipopt::Number T, Ipopt::Number* cop)
	{
		const Ipopt::Number* left_foot = p->pfL;
		const Ipopt::Number* right_foot = p->pfR;
		const Ipopt::Number* lam0 = p->lam0;
		const Ipopt::Number* lamT = p->lamT;

		Ipopt::Number lam[] = {0.0, 0.0, 0.0, 0.0};
		for (int i = 0; i < 4; i++)
			lam[i] = lam0[i]*(1 - t/T) + lamT[i]*(t/T);

		Ipopt::Number caL = cos(left_foot[2]);
		Ipopt::Number saL = sin(left_foot[2]);
		Ipopt::Number caR = cos(right_foot[2]);
		Ipopt::Number saR = sin(right_foot[2]);

		cop[0] = lam[0]*(left_foot[0] + cx[0]*caL) +
				lam[1]*(left_foot[0] + cx[1]*caL) +
				lam[2]*(right_foot[0] + cx[0]*caR) +
				lam[3]*(right_foot[0] + cx[1]*caR);

		cop[1] = lam[0]*(left_foot[1] + cx[0]*saL) +
				lam[1]*(left_foot[1] + cx[1]*saL) +
				lam[2]*(right_foot[1] + cx[0]*saR) +
				lam[3]*(right_foot[1] + cx[1]*saR);
	}

	void getXY(const PHASE_Params* s, Ipopt::Number T, Ipopt::Number* xy)
	{
		Ipopt::Number p0[] = {0.0, 0.0};
		calc_cop(s, 0.0, T, p0);
		Ipopt::Number pT[] = {0.0, 0.0};
		calc_cop(s, T, T, pT);
		Ipopt::Number alpha = sqrt(g / s->c0[2]);

		for (int i = 0; i < 2; i++)
		{
			Ipopt::Number B1 = (s->c0[i] - p0[i])/2.0 + (s->c0[i+4]*T - (pT[i] - p0[i]))/(2*alpha*T);
			Ipopt::Number B2 = (s->c0[i] - p0[i])/2.0 - (s->c0[i+4]*T - (pT[i] - p0[i]))/(2*alpha*T);
			xy[i] = B1*exp(alpha*T) + B2*exp(-alpha*T) + pT[i];
			xy[i+2] = alpha*B1*exp(alpha*T) - alpha*B2*exp(-alpha*T) + (1/T)*(pT[i] - p0[i]);
		}
	}

	void getZ(const PHASE_Params* s, int phase_idx, Ipopt::Number* z)
	{
		Ipopt::Number T = phase[phase_idx].T;
		Ipopt::Number d1 = s->c0[2] - s->r0 + g_omega_2;
		Ipopt::Number d2 = s->c0[6]/omega - (s->rT - s->r0)/(T*omega);
		z[0] = d1*cwT[phase_idx] + d2*swT[phase_idx] + s->rT - g_omega_2;
		z[1] = -d1*omega*swT[phase_idx] + d2*omega*cwT[phase_idx] + (1/T)*(s->rT-s->r0);
	}

	void getXY_at_time(const PHASE_Params* s, Ipopt::Number T, Ipopt::Number* xy, Ipopt::Number t)
	{
		Ipopt::Number p0[] = {0.0, 0.0};
		calc_cop(s, 0.0, T, p0);
		Ipopt::Number pT[] = {0.0, 0.0};
		calc_cop(s, T, T, pT);
		Ipopt::Number pt[] = {0.0, 0.0};
		calc_cop(s, t, T, pt);
		Ipopt::Number alpha = sqrt(g / s->c0[2]);

		for (int i = 0; i < 2; i++)
		{
			Ipopt::Number B1 = (s->c0[i] - p0[i])/2.0 + (s->c0[i+4]*T - (pT[i] - p0[i]))/(2*alpha*T);
			Ipopt::Number B2 = (s->c0[i] - p0[i])/2.0 - (s->c0[i+4]*T - (pT[i] - p0[i]))/(2*alpha*T);
			xy[i] = B1*exp(alpha*t) + B2*exp(-alpha*t) + pt[i];
			xy[i+2] = alpha*B1*exp(alpha*t) - alpha*B2*exp(-alpha*t) + (1/T)*(pT[i] - p0[i]);
			xy[i+4] = alpha*alpha*B1*exp(alpha*t) + alpha*alpha*B2*exp(-alpha*t);
		}
	}

	void getZ_at_time(const PHASE_Params* s, int phase_idx, Ipopt::Number* z, Ipopt::Number t)
	{
		Ipopt::Number T = phase[phase_idx].T;
		Ipopt::Number d1 = s->c0[2] - s->r0 + g_omega_2;
		Ipopt::Number d2 = s->c0[6]/omega - (s->rT - s->r0)/(T*omega);
		z[0] = d1*cos(omega*t) + d2*sin(omega*t) + (t/T)*(s->rT - s->r0) + s->r0 - g_omega_2;
		z[1] = -d1*omega*sin(omega*t) + d2*omega*cos(omega*t) + (1/T)*(s->rT-s->r0);
		z[2] = -d1*omega*omega*cos(omega*t) - d2*omega*omega*sin(omega*t);
	}

	void getZjacobian(const PHASE_Params* s, int phase_idx, Ipopt::Number* dz)
	{
		Ipopt::Number T = phase[phase_idx].T;
		dz[0] = cwT[phase_idx];
		dz[1] = swT[phase_idx]/omega;
		dz[2] = -cwT[phase_idx] + swT[phase_idx]/(T*omega);
		dz[3] = -swT[phase_idx]/(omega*T) + 1;
		dz[4] = -omega*swT[phase_idx];
		dz[5] = cwT[phase_idx];
		dz[6] = omega*swT[phase_idx] + cwT[phase_idx]/T - 1/T;
		dz[7] = -cwT[phase_idx]/T + 1/T;
	}

	void getA(const PHASE_Params* s, Ipopt::Number T, Ipopt::Number* a)
	{
		a[1] = s->c0[7] + s->add*T;
		a[0] = s->c0[3] + s->c0[7]*T + 0.5*s->add*T*T;
	}

	void getCOPjacobian(const PHASE_Params* s, Ipopt::Number* dp0dl, Ipopt::Number* dp0dpL, Ipopt::Number* dp0dpR, Ipopt::Number* dpTdl, Ipopt::Number* dpTdpL, Ipopt::Number* dpTdpR)
	{
		const Ipopt::Number* left_foot = s->pfL;
		const Ipopt::Number* right_foot = s->pfR;
		const Ipopt::Number* lam0 = s->lam0;
		const Ipopt::Number* lamT = s->lamT;

		Ipopt::Number caL = cos(left_foot[2]);
		Ipopt::Number saL = sin(left_foot[2]);
		Ipopt::Number caR = cos(right_foot[2]);
		Ipopt::Number saR = sin(right_foot[2]);

		//dp0dl
		dp0dl[0] = left_foot[0] + cx[0]*caL;
		dp0dl[1] = left_foot[1] + cx[0]*saL;
		dp0dl[2] = left_foot[0] + cx[1]*caL;
		dp0dl[3] = left_foot[1] + cx[1]*saL;
		dp0dl[4] = right_foot[0] + cx[0]*caR;
		dp0dl[5] = right_foot[1] + cx[0]*saR;
		dp0dl[6] = right_foot[0] + cx[1]*caR;
		dp0dl[7] = right_foot[1] + cx[1]*saR;
		for (int i = 0; i < 8; i++)
			dpTdl[i] = dp0dl[i];

		//dp0dpL
		dp0dpL[0] = lam0[0] + lam0[1];
		dp0dpL[1] = lam0[0] + lam0[1];
		dp0dpL[2] = -(lam0[0]*cx[0]*saL + lam0[1]*cx[1]*saL);
		dp0dpL[3] = (lam0[0]*cx[0]*caL + lam0[1]*cx[1]*caL);
		//dpTdpL
		dpTdpL[0] = lamT[0] + lamT[1];
		dpTdpL[1] = lamT[0] + lamT[1];
		dpTdpL[2] = -(lamT[0]*cx[0]*saL + lamT[1]*cx[1]*saL);
		dpTdpL[3] = (lamT[0]*cx[0]*caL + lamT[1]*cx[1]*caL);

		//dp0dpR
		dp0dpR[0] = lam0[2] + lam0[3];
		dp0dpR[1] = lam0[2] + lam0[3];
		dp0dpR[2] = -(lam0[2]*cx[0]*saR + lam0[3]*cx[1]*saR);
		dp0dpR[3] = (lam0[2]*cx[0]*caR + lam0[3]*cx[1]*caR);
		//dpTdpR
		dpTdpR[0] = lamT[2] + lamT[3];
		dpTdpR[1] = lamT[2] + lamT[3];
		dpTdpR[2] = -(lamT[2]*cx[0]*saR + lamT[3]*cx[1]*saR);
		dpTdpR[3] = (lamT[2]*cx[0]*caR + lamT[3]*cx[1]*caR);

	}

	void getXYjacobian(const PHASE_Params* s, Ipopt::Number T, Ipopt::Number* xy)
	{
		Ipopt::Number p0[] = {0.0, 0.0};
		calc_cop(s, 0.0, T, p0);
		Ipopt::Number pT[] = {0.0, 0.0};
		calc_cop(s, T, T, pT);
		Ipopt::Number alpha = sqrt(g / s->c0[2]);
		Ipopt::Number eaT = exp(alpha*T);
		Ipopt::Number enaT = exp(-alpha*T);

		//Beta1,2 for x and y
		Ipopt::Number B1[4];
		B1[0] = 0.5*(s->c0[0] - p0[0]);
		B1[1] = 0.5*(s->c0[1] - p0[1]);
		B1[2] = (s->c0[4]*T - (pT[0] - p0[0]))/(2*alpha*T);
		B1[3] = (s->c0[5]*T - (pT[1] - p0[1]))/(2*alpha*T);
		Ipopt::Number B2[4];
		B2[0] = B1[0];
		B2[1] = B1[1];
		B2[2] = -B1[2];
		B2[3] = -B1[3];

		//xy should be 4*15

		//dx/dx0 and dy/dy0
		xy[0] = 0.5*eaT + 0.5*enaT;
		xy[15] = xy[0];
		//dxd/dx0 and dyd/dy0
		xy[30] = 0.5*alpha*eaT - 0.5*alpha*enaT;
		xy[45] = xy[30];

		//dx/dz0 and dy/dz0
		xy[1] = 0.5*(B1[2]/s->c0[2])*eaT + (-0.5*alpha*T/s->c0[2])*(B1[0]+B1[2])*eaT +
				0.5*(B2[2]/s->c0[2])*enaT + (0.5*alpha*T/s->c0[2])*(B2[0]+B2[2])*enaT;
		xy[16] = 0.5*(B1[3]/s->c0[2])*eaT + (-0.5*alpha*T/s->c0[2])*(B1[1]+B1[3])*eaT +
				0.5*(B2[3]/s->c0[2])*enaT + (0.5*alpha*T/s->c0[2])*(B2[1]+B2[3])*enaT;
		//dxd/dz0 and dyd/dz0
		xy[31] = -(0.5*B1[0]*alpha/s->c0[2])*eaT + (-0.5*alpha*T/s->c0[2])*alpha*(B1[0]+B1[2])*eaT +
				+ (0.5*B2[0]*alpha/s->c0[2])*enaT + (-0.5*alpha*T/s->c0[2])*alpha*(B2[0]+B2[2])*enaT;
		xy[46] = -(0.5*B1[1]*alpha/s->c0[2])*eaT + (-0.5*alpha*T/s->c0[2])*alpha*(B1[1]+B1[3])*eaT +
						+ (0.5*B2[1]*alpha/s->c0[2])*enaT + (-0.5*alpha*T/s->c0[2])*alpha*(B2[1]+B2[3])*enaT;

		//dx/dxd0 and dy/dyd0
		xy[2] = (0.5/alpha)*eaT - (0.5/alpha)*enaT;
		xy[17] = xy[2];
		//dxd/dxd0 and dyd/dyd0
		xy[32] = 0.5*eaT + 0.5*enaT;
		xy[47] = xy[32];

		Ipopt::Number dp0dl[8];
		Ipopt::Number dp0dpL[4];
		Ipopt::Number dp0dpR[4];
		Ipopt::Number dpTdl[8];
		Ipopt::Number dpTdpL[4];
		Ipopt::Number dpTdpR[4];

		getCOPjacobian(s, dp0dl, dp0dpL, dp0dpR, dpTdl, dpTdpL, dpTdpR);

		for (int i = 0; i < 2; i++)
		{
			xy[3+i] = (-0.5*dp0dpL[i*2] - 0.5*(dpTdpL[i*2] - dp0dpL[i*2])/(alpha*T))*eaT +
					(-0.5*dp0dpL[i*2] + 0.5*(dpTdpL[i*2] - dp0dpL[i*2])/(alpha*T))*enaT + dpTdpL[i*2];
			xy[18+i] = (-0.5*dp0dpL[i*2+1] - 0.5*(dpTdpL[i*2+1] - dp0dpL[i*2+1])/(alpha*T))*eaT +
					(-0.5*dp0dpL[i*2+1] + 0.5*(dpTdpL[i*2+1] - dp0dpL[i*2+1])/(alpha*T))*enaT + dpTdpL[i*2+1];
			xy[33+i] = alpha*(-0.5*dp0dpL[i*2] - 0.5*(dpTdpL[i*2] - dp0dpL[i*2])/(alpha*T))*eaT +
					-alpha*(-0.5*dp0dpL[i*2] + 0.5*(dpTdpL[i*2] - dp0dpL[i*2])/(alpha*T))*enaT + (1/T)*(dpTdpL[i*2]-dp0dpL[i*2]);
			xy[48+i] = alpha*(-0.5*dp0dpL[i*2+1] - 0.5*(dpTdpL[i*2+1] - dp0dpL[i*2+1])/(alpha*T))*eaT +
					-alpha*(-0.5*dp0dpL[i*2+1] + 0.5*(dpTdpL[i*2+1] - dp0dpL[i*2+1])/(alpha*T))*enaT + (1/T)*(dpTdpL[i*2+1]-dp0dpL[i*2+1]);
		}

		for (int i = 0; i < 2; i++)
		{
			xy[5+i] = (-0.5*dp0dpR[i*2] - 0.5*(dpTdpR[i*2] - dp0dpR[i*2])/(alpha*T))*eaT +
					(-0.5*dp0dpR[i*2] + 0.5*(dpTdpR[i*2] - dp0dpR[i*2])/(alpha*T))*enaT + dpTdpR[i*2];
			xy[20+i] = (-0.5*dp0dpR[i*2+1] - 0.5*(dpTdpR[i*2+1] - dp0dpR[i*2+1])/(alpha*T))*eaT +
					(-0.5*dp0dpR[i*2+1] + 0.5*(dpTdpR[i*2+1] - dp0dpR[i*2+1])/(alpha*T))*enaT + dpTdpR[i*2+1];
			xy[35+i] = alpha*(-0.5*dp0dpR[i*2] - 0.5*(dpTdpR[i*2] - dp0dpR[i*2])/(alpha*T))*eaT +
					-alpha*(-0.5*dp0dpR[i*2] + 0.5*(dpTdpR[i*2] - dp0dpR[i*2])/(alpha*T))*enaT + (1/T)*(dpTdpR[i*2]-dp0dpR[i*2]);
			xy[50+i] = alpha*(-0.5*dp0dpR[i*2+1] - 0.5*(dpTdpR[i*2+1] - dp0dpR[i*2+1])/(alpha*T))*eaT +
					-alpha*(-0.5*dp0dpR[i*2+1] + 0.5*(dpTdpR[i*2+1] - dp0dpR[i*2+1])/(alpha*T))*enaT + (1/T)*(dpTdpR[i*2+1]-dp0dpR[i*2+1]);
		}

		for (int i = 0; i < 4; i++)
		{
			xy[7+i] = (-0.5*dp0dl[i*2] + 0.5*dp0dl[i*2]/(alpha*T))*eaT +
					(-0.5*dp0dl[i*2] - 0.5*dp0dl[i*2]/(alpha*T))*enaT;
			xy[22+i] = (-0.5*dp0dl[i*2+1] + 0.5*dp0dl[i*2+1]/(alpha*T))*eaT +
					(-0.5*dp0dl[i*2+1] - 0.5*dp0dl[i*2+1]/(alpha*T))*enaT;
			xy[37+i] = alpha*(-0.5*dp0dl[i*2] + 0.5*dp0dl[i*2]/(alpha*T))*eaT +
					-alpha*(-0.5*dp0dl[i*2] - 0.5*dp0dl[i*2]/(alpha*T))*enaT - (1/T)*dp0dl[i*2];
			xy[52+i] = alpha*(-0.5*dp0dl[i*2+1] + 0.5*dp0dl[i*2+1]/(alpha*T))*eaT +
					-alpha*(-0.5*dp0dl[i*2+1] - 0.5*dp0dl[i*2+1]/(alpha*T))*enaT - (1/T)*dp0dl[i*2+1];
		}

		for (int i = 0; i < 4; i++)
		{
			xy[11+i] = (-0.5*dpTdl[i*2]/(alpha*T))*eaT +
					(0.5*dpTdl[i*2]/(alpha*T))*enaT + dpTdl[i*2];
			xy[26+i] = (-0.5*dpTdl[i*2+1]/(alpha*T))*eaT +
					(0.5*dpTdl[i*2+1]/(alpha*T))*enaT + dpTdl[i*2+1];
			xy[41+i] = alpha*(-0.5*dpTdl[i*2]/(alpha*T))*eaT +
					-alpha*(0.5*dpTdl[i*2]/(alpha*T))*enaT + (1/T)*dpTdl[i*2];
			xy[56+i] = alpha*(-0.5*dpTdl[i*2+1]/(alpha*T))*eaT +
					-alpha*(0.5*dpTdl[i*2+1]/(alpha*T))*enaT + (1/T)*dpTdl[i*2+1];
		}

	}


	int N; //num open vars
	int M; //num constraints
	int Mc_idx;
	int Mf_idx;
	int Mp_idx;
	int Mr_idx;
	int Ml_idx;
	int Mp_nl_idx;
	int Mp_a_idx;

	int num_phases;

	PHASE_Info phase[MAX_NUM_PHASES];

	Ipopt::Number x0[14]; //start state
	Ipopt::Number xT[8]; //target positions at the end

	std::vector<std::vector<Ipopt::Index>> foot_equalities;

	static constexpr double g = 9.806;
	static constexpr double stiffness = 1000;
	static constexpr double mass = 31.0;

	static constexpr double cx[] = {-0.079, 0.079};
	static constexpr double foot_ext[] = {0.25, 0.075};
	static constexpr double foot_nom[] = {0.0, 0.13, 0.0, -0.13};

	//auxilary variables that you only need to calc once
	Ipopt::Number cwT[MAX_NUM_PHASES];
	Ipopt::Number swT[MAX_NUM_PHASES];
	Ipopt::Number omega;
	Ipopt::Number g_omega_2;
};

namespace Ipopt {
class MPC_NLP : public TNLP
{
public:
	/** default constructor */
	MPC_NLP();

	/** default destructor */
	virtual ~MPC_NLP();

	/**@name Overloaded from TNLP */
	//@{
	/** Method to return some info about the nlp */
	virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
			Ipopt::Index& nnz_h_lag, IndexStyleEnum& Index_style);

	/** Method to return the bounds for my problem */
	virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
			Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

	/** Method to return the starting point for the algorithm */
	virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
			bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
			Ipopt::Index m, bool init_lambda,
			Ipopt::Number* lambda);

	/** Method to return the objective value */
	virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value);

	/** Method to return the gradient of the objective */
	virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f);

	/** Method to return the constraint residuals */
	virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g);

	/** Method to return:
	 *   1) The structure of the jacobian (if "values" is NULL)
	 *   2) The values of the jacobian (if "values" is not NULL)
	 */
	virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
			Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
			Ipopt::Number* values);

	/** Method to return:
	 *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
	 *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
	 */
	virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
			Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
			bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
			Ipopt::Index* jCol, Ipopt::Number* values);

	//@}

	/** @name Solution Methods */
	//@{
	/** This method is called when the algorithm is complete so the TNLP can store/write the solution */
	virtual void finalize_solution(Ipopt::SolverReturn status,
			Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
			Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
			Ipopt::Number obj_value,
			const Ipopt::IpoptData* ip_data,
			Ipopt::IpoptCalculatedQuantities* ip_cq);
	//@}

	void Setup(MPC_OPTIONS* Opt, bool bWarmStart);

	void GetSolution(ROM_Policy_Struct* targ_traj, double dt);
private:
	/**@name Methods to block default compiler methods.
	 * The compiler automatically generates the following three methods.
	 *  Since the default compiler implementation is generally not what
	 *  you want (for all but the most simple classes), we usually
	 *  put the declarations of these methods in the private section
	 *  and never implement them. This prevents the compiler from
	 *  implementing an incorrect "default" behavior without us
	 *  knowing. (See Scott Meyers book, "Effective C++")
	 *
	 */
	//@{
	//  MPC_NLP();
	MPC_NLP(const MPC_NLP&);
	MPC_NLP& operator=(const MPC_NLP&);

	void get_hessian_structure(Ipopt::Index n, Ipopt::Index* iRow, Ipopt::Index* jCol);
	void get_jacobian_structure(Ipopt::Index n, Ipopt::Index m, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Index nele_jac);

	MPC_OPTIONS* opt;

	bool m_bWarmStart;

	// Ipopt interprets any Ipopt::Number greater than nlp_upper_bound_inf as
	// infinity. The default value of nlp_upper_bound_inf and nlp_lower_bound_inf
	// is 1e19 and can be changed through ipopt options.
	static constexpr Ipopt::Number NLP_INF = 2.0e19;

	static constexpr double m_dAlphaW = 1e-1;
	static constexpr double m_dJerkW = 1e-6;

	Ipopt::Number x_last[MAX_IPOPT_VARS];

	//@}
};
};

#endif
