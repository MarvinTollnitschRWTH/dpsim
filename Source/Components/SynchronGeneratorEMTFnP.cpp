/** Synchron Generator (EMTnP)
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include "SynchronGeneratorEMTFnP.h"
#include "IntegrationMethod.h"

using namespace DPsim;

SynchronGeneratorEMTFnP::SynchronGeneratorEMTFnP(std::string name, Int node1, Int node2, Int node3,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia, bool logActive)
	: BaseComponent(name, node1, node2, node3, logActive) {

	mNomPower = nomPower;
	mNomVolt = nomVolt;
	mNomFreq = nomFreq;
	mPoleNumber = poleNumber;
	mNomFieldCur = nomFieldCur;

	// base stator values
	mBase_V_RMS = mNomVolt / sqrt(3);
	mBase_v = mBase_V_RMS * sqrt(2);
	mBase_I_RMS = mNomPower / (3 * mBase_V_RMS);
	mBase_i = mBase_I_RMS * sqrt(2);
	mBase_Z = mBase_v / mBase_i;
	mBase_OmElec = 2 * DPS_PI * mNomFreq;
	mBase_OmMech = mBase_OmElec / (mPoleNumber / 2);
	mBase_L = mBase_Z / mBase_OmElec;
	mBase_Psi = mBase_L * mBase_i;
	mBase_T = mNomPower / mBase_OmMech;

	// Create logging file
	if (mLogActive) {
		std::string filename = "SynGen_" + mName + ".csv";
		mLog = new Logger(filename);
	}

	// steady state per unit initial value
	initWithPerUnitParam(Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);

}


SynchronGeneratorEMTFnP::~SynchronGeneratorEMTFnP() {
	if (mLogActive) {
		delete mLog;
	}
}

void SynchronGeneratorEMTFnP::initWithPerUnitParam(
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real H) {

	if (Rkq2 == 0 && Llkq2 == 0)
	{
		DampingWindings = 1;
		mVoltages2 = Matrix::Zero(6, 1);
		mFluxes2 = Matrix::Zero(6, 1);
		mCurrents2 = Matrix::Zero(6, 1);
	}
	else
	{
		DampingWindings = 2;
	}


	// base rotor values
	mBase_ifd = Lmd * mNomFieldCur;
	mBase_vfd = mNomPower / mBase_ifd;
	mBase_Zfd = mBase_vfd / mBase_ifd;
	mBase_Lfd = mBase_Zfd / mBase_OmElec;

	mRs = Rs;
	mLl = Ll;
	mLmd = Lmd;
	mLmd0 = Lmd0;
	mLmq = Lmq;
	mLmq0 = Lmq0;
	mRfd = Rfd;
	mLlfd = Llfd;
	mRkd = Rkd;
	mLlkd = Llkd;
	mRkq1 = Rkq1;
	mLlkq1 = Llkq1;
	mRkq2 = Rkq2;
	mLlkq2 = Llkq2;
	mH = H;
	// Additional inductances according to Krause
	mLaq = 1 / (1 / mLmq + 1 / mLl + 1 / mLlkq1 + 1 / mLlkq2);
	mLad = 1 / (1 / mLmd + 1 / mLl + 1 / mLlkd + 1 / mLlfd);

	// Determinant of Ld (inductance matrix of d axis)
	detLd = (mLmd + mLl)*(-mLlfd*mLlkd - mLlfd*mLmd - mLmd*mLlkd) + mLmd*mLmd*(mLlfd + mLlkd);
	// Determinant of Lq (inductance matrix of q axis)
	if (DampingWindings == 2)
		detLq = -mLmq*mLlkq2*(mLlkq1 + mLl) - mLl*mLlkq1*(mLlkq2 + mLmq);
	else
		detLq = -(mLl + mLmq)*(mLlkq1 + mLmq) + mLmq*mLmq;
}

void SynchronGeneratorEMTFnP::init(Real om, Real dt,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt,
	Real initVoltAngle, Real initFieldVoltage, Real initMechPower) {

	// Create matrices for state space representation
	if (DampingWindings == 2)
	{
		mInductanceMat <<
			-(mLl + mLmq), 0, 0, mLmq, mLmq, 0, 0,
			0, -(mLl + mLmd), 0, 0, 0, mLmd, mLmd,
			0, 0, -mLl, 0, 0, 0, 0,
			-mLmq, 0, 0, mLlkq1 + mLmq, mLmq, 0, 0,
			-mLmq, 0, 0, mLmq, mLlkq2 + mLmq, 0, 0,
			0, -mLmd, 0, 0, 0, mLlfd + mLmd, mLmd,
			0, -mLmd, 0, 0, 0, mLmd, mLlkd + mLmd;

		mResistanceMat <<
			mRs, 0, 0, 0, 0, 0, 0,
			0, mRs, 0, 0, 0, 0, 0,
			0, 0, mRs, 0, 0, 0, 0,
			0, 0, 0, -mRkq1, 0, 0, 0,
			0, 0, 0, 0, -mRkq2, 0, 0,
			0, 0, 0, 0, 0, -mRfd, 0,
			0, 0, 0, 0, 0, 0, -mRkd;

		mOmegaFluxMat <<
			0, 1, 0, 0, 0, 0, 0,
			-1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0;
	}
	else
	{
		mInductanceMat = Matrix::Zero(6, 6);
		mResistanceMat = Matrix::Zero(6, 6);
		mReactanceMat = Matrix::Zero(6, 6);
		mOmegaFluxMat = Matrix::Zero(6, 6);

		mInductanceMat <<
			-(mLl + mLmq), 0, 0, mLmq, 0, 0,
			0, -(mLl + mLmd), 0, 0, mLmd, mLmd,
			0, 0, -mLl, 0, 0, 0,
			-mLmq, 0, 0, mLlkq1 + mLmq, 0, 0,
			0, -mLmd, 0, 0, mLlfd + mLmd, mLmd,
			0, -mLmd, 0, 0, mLmd, mLlkd + mLmd;
		mResistanceMat <<
			mRs, 0, 0, 0, 0, 0,
			0, mRs, 0, 0, 0, 0,
			0, 0, mRs, 0, 0, 0,
			0, 0, 0, -mRkq1, 0, 0,
			0, 0, 0, 0, -mRfd, 0,
			0, 0, 0, 0, 0, -mRkd;

		mOmegaFluxMat <<
			0, 1, 0, 0, 0, 0,
			-1, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0;
	}

	mReactanceMat = mInductanceMat.inverse();


	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);

	mVa = inverseParkTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(0);
	mVb = inverseParkTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(1);
	mVc = inverseParkTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(2);

	mIa = inverseParkTransform2(mThetaMech, mId* mBase_i, mIq* mBase_i, mI0* mBase_i)(0);
	mIb = inverseParkTransform2(mThetaMech, mId* mBase_i, mIq* mBase_i, mI0* mBase_i)(1);
	mIc = inverseParkTransform2(mThetaMech, mId* mBase_i, mIq* mBase_i, mI0* mBase_i)(2);
}

void SynchronGeneratorEMTFnP::initStatesInPerUnit(Real initActivePower, Real initReactivePower,
	Real initTerminalVolt, Real initVoltAngle, Real initFieldVoltage, Real initMechPower) {

	// #### Electrical variables ##############################################
	Real init_P = initActivePower / mNomPower;
	Real init_Q = initReactivePower / mNomPower;
	Real init_S = sqrt(pow(init_P, 2.) + pow(init_Q, 2.));
	Real init_vt = initTerminalVolt / mBase_v;
	Real init_it = init_S / init_vt;

	// power factor
	Real init_pf = acos(init_P / init_S);

	// load angle
	Real init_delta = atan(((mLmq + mLl) * init_it * cos(init_pf) - mRs * init_it * sin(init_pf)) /
		(init_vt + mRs * init_it * cos(init_pf) + (mLmq + mLl) * init_it * sin(init_pf)));
	Real init_delta_deg = init_delta / DPS_PI * 180;

	// dq stator voltages and currents
	Real init_vd = init_vt * sin(init_delta);
	Real init_vq = init_vt * cos(init_delta);
	Real init_id = init_it * sin(init_delta + init_pf);
	Real init_iq = init_it * cos(init_delta + init_pf);

	// rotor voltage and current
	Real init_ifd = (init_vq + mRs * init_iq + (mLmd + mLl) * init_id) / mLmd;
	Real init_vfd = mRfd * init_ifd;

	// flux linkages
	Real init_psid = init_vq + mRs * init_iq;
	Real init_psiq = -init_vd - mRs * init_id;
	Real init_psifd = (mLmd + mLlfd) * init_ifd - mLmd * init_id;
	Real init_psid1 = mLmd * (init_ifd - init_id);
	Real init_psiq1 = -mLmq * init_iq;
	Real init_psiq2 = -mLmq * init_iq;

	// rotor mechanical variables
	Real init_Te = init_P + mRs * pow(init_it, 2.);
	mOmMech = 1;

	mVd = init_vd;
	mVq = init_vq;
	mV0 = 0;
	mVfd = init_vfd;
	mVkd = 0;
	mVkq1 = 0;
	mVkq2 = 0;

	mIq = init_iq;
	mId = init_id;
	mI0 = 0;
	mIfd = init_ifd;
	mIkd = 0;
	mIkq1 = 0;
	mIkq2 = 0;

	mPsiq = init_psiq;
	mPsid = init_psid;
	mPsi0 = 0;
	mPsifd = init_psifd;
	mPsikd = init_psid1;
	mPsikq1 = init_psiq1;
	mPsikq2 = init_psiq2;

	// #### mechanical variables ##############################################
	mMechPower = initMechPower / mNomPower;
	mMechTorque = mMechPower / 1;
	mThetaMech = initVoltAngle + init_delta - M_PI / 2;
	//mThetaMech = initVoltAngle + init_delta;
}

void SynchronGeneratorEMTFnP::step(SystemModel& system, Real time) {

	stepInPerUnit(system.getOmega(), system.getTimeStep(), time, system.getNumMethod());

	// Update current source accordingly
	if (mNode1 >= 0) {
		system.addRealToRightSideVector(mNode1, mIa);
	}
	if (mNode2 >= 0) {
		system.addRealToRightSideVector(mNode2, mIb);
	}
	if (mNode3 >= 0) {
		system.addRealToRightSideVector(mNode3, mIc);
	}

	if (mLogActive) {
		Matrix logValues(getFluxes().rows() + getVoltages().rows() + getCurrents().rows(), 1);
		logValues << getFluxes(), getVoltages(), getCurrents();
		mLog->LogDataLine(time, logValues);
	}

}

Real SynchronGeneratorEMTFnP::dtOmMech(Matrix inputs) {

	return  1 / (2 * inputs(2)) * (inputs(1) - inputs(0));
}

Real SynchronGeneratorEMTFnP::dtThetaMech(Matrix inputs) {

	return  inputs(0);
}


void SynchronGeneratorEMTFnP::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod) {

	mVa = (1 / mBase_v) * mVa;
	mVb = (1 / mBase_v) * mVb;
	mVc = (1 / mBase_v) * mVc;

	mIa = (1 / mBase_i) * mIa;
	mIb = (1 / mBase_i) * mIb;
	mIc = (1 / mBase_i) * mIc;

	// dq-transform of interface voltage
	mVd = parkTransform2(mThetaMech, mVa, mVb, mVc)(0);
	mVq = parkTransform2(mThetaMech, mVa, mVb, mVc)(1);
	mV0 = parkTransform2(mThetaMech, mVa, mVb, mVc)(2);

	mElecTorque = (mPsid*mIq - mPsiq*mId);

	// Euler step forward for rotational speed
	mOmMechInputs <<
		mElecTorque,
		mMechTorque,
		mH;
	mOmMech = Euler(mOmMech, mOmMechInputs, SynchronGeneratorEMTFnP::dtOmMech, dt);

		if (DampingWindings == 2)
		{

			Matrix Fluxes(7, 1);
			Fluxes(0, 0) = mPsiq;
			Fluxes(1, 0) = mPsid;
			Fluxes(2, 0) = mPsi0;
			Fluxes(3, 0) = mPsikq1;
			Fluxes(4, 0) = mPsikq2;
			Fluxes(5, 0) = mPsifd;
			Fluxes(6, 0) = mPsikd;

			Matrix dqVoltages(7, 1);
			dqVoltages(0, 0) = mVq;
			dqVoltages(1, 0) = mVd;
			dqVoltages(2, 0) = mV0;
			dqVoltages(3, 0) = mVkq1;
			dqVoltages(4, 0) = mVkq2;
			dqVoltages(5, 0) = mVfd;
			dqVoltages(6, 0) = mVkd;

			Matrix A = (mResistanceMat*mReactanceMat - mOmMech*mOmegaFluxMat);
			Matrix B = Matrix::Identity(7, 7);

			if (numMethod == NumericalMethod::Trapezoidal_flux)
			{
				Fluxes = Trapezoidal(Fluxes, A, B, dt*mBase_OmElec, dqVoltages);
			}
			else if (numMethod == NumericalMethod::Euler)
			{	
				Fluxes = Euler(Fluxes, A, B, dt*mBase_OmElec, dqVoltages);
			}
			

			mPsiq = Fluxes(0, 0);
			mPsid = Fluxes(1, 0);
			mPsi0 = Fluxes(2, 0);
			mPsikq1 = Fluxes(3, 0);
			mPsikq2 = Fluxes(4, 0);
			mPsifd = Fluxes(5, 0);
			mPsikd = Fluxes(6, 0);

		}
		else

		{
			Matrix A = (mResistanceMat*mReactanceMat - mOmMech*mOmegaFluxMat);
			Matrix B = Matrix::Identity(6, 6);

			Matrix Fluxes(6, 1);
			Fluxes(0, 0) = mPsiq;
			Fluxes(1, 0) = mPsid;
			Fluxes(2, 0) = mPsi0;
			Fluxes(3, 0) = mPsikq1;
			Fluxes(4, 0) = mPsifd;
			Fluxes(5, 0) = mPsikd;

			Matrix dqVoltages(6, 1);
			dqVoltages(0, 0) = mVq;
			dqVoltages(1, 0) = mVd;
			dqVoltages(2, 0) = mV0;
			dqVoltages(3, 0) = mVkq1;
			dqVoltages(4, 0) = mVfd;
			dqVoltages(5, 0) = mVkd;

			if (numMethod == NumericalMethod::Trapezoidal_flux)
			{
				Fluxes = Trapezoidal(Fluxes, A, B, dt*mBase_OmElec, dqVoltages);
			}
			else if (numMethod == NumericalMethod::Euler)
			{	
				Fluxes = Euler(Fluxes, A, B, dt*mBase_OmElec, dqVoltages);
			}

			mPsiq = Fluxes(0, 0);
			mPsid = Fluxes(1, 0);
			mPsi0 = Fluxes(2, 0);
			mPsikq1 = Fluxes(3, 0);
			mPsifd = Fluxes(4, 0);
			mPsikd = Fluxes(5, 0);

		}

		//Calculation of currents based on inverse of inductance matrix
		mId = ((mLlfd*mLlkd + mLmd*(mLlfd + mLlkd))*mPsid - mLmd*mLlkd*mPsifd - mLlfd*mLmd*mPsikd) / detLd;
		mIfd = (mLlkd*mLmd*mPsid - (mLl*mLlkd + mLmd*(mLl + mLlkd))*mPsifd + mLmd*mLl*mPsikd) / detLd;
		mIkd = (mLmd*mLlfd*mPsid + mLmd*mLl*mPsifd - (mLmd*(mLlfd + mLl) + mLl*mLlfd)*mPsikd) / detLd;
		if (DampingWindings == 2)
		{
			mIq = ((mLlkq1*mLlkq2 + mLmq*(mLlkq1 + mLlkq2))*mPsiq - mLmq*mLlkq2*mPsikq1 - mLmq*mLlkq1*mPsikq2) / detLq;
			mIkq1 = (mLmq*mLlkq2*mPsiq - (mLmq*(mLlkq2 + mLl) + mLl*mLlkq2)*mPsikq1 + mLmq*mLl*mPsikq2) / detLq;
			mIkq2 = (mLmq*mLlkq1*mPsiq + mLmq*mLl*mPsikq1 - (mLmq*(mLlkq1 + mLl) + mLl*mLlkq1)*mPsikq2) / detLq;
		}
		else
		{
			mIq = ((mLlkq1 + mLmq)*mPsiq - mLmq*mPsikq1) / detLq;
			mIkq1 = (mLmq*mPsiq - (mLl + mLmq)*mPsikq1) / detLq;
		}
		mI0 = -mPsi0 / mLl;

		mOmMech_Matrix <<
			mOmMech*mBase_OmMech;
		mThetaMech = Euler(mThetaMech, mOmMech_Matrix, SynchronGeneratorEMTFnP::dtThetaMech, dt);

	mIa = mBase_i * inverseParkTransform2(mThetaMech, mId, mIq, mI0)(0);
	mIb = mBase_i * inverseParkTransform2(mThetaMech, mId, mIq, mI0)(1);
	mIc = mBase_i * inverseParkTransform2(mThetaMech, mId, mIq, mI0)(2);

	if (DampingWindings == 2)
	{
		mCurrents2 << mIq,
			mId,
			mI0,
			mIkq1,
			mIkq2,
			mIfd,
			mIkd;

		mVoltages2 << mVq,
			mVd,
			mV0,
			mVkq1,
			mVkq2,
			mVfd,
			mVkd;

		mFluxes2 << mPsiq,
			mPsid,
			mPsi0,
			mPsikq1,
			mPsikq2,
			mPsifd,
			mPsikd;
	}
	else
	{
		mCurrents2 << mIq,
			mId,
			mI0,
			mIkq1,
			mIfd,
			mIkd;

		mVoltages2 << mVq,
			mVd,
			mV0,
			mVkq1,
			mVfd,
			mVkd;

		mFluxes2 << mPsiq,
			mPsid,
			mPsi0,
			mPsikq1,
			mPsifd,
			mPsikd;
	}
}

void SynchronGeneratorEMTFnP::postStep(SystemModel& system) {
	if (mNode1 >= 0) {
		mVa = system.getRealFromLeftSideVector(mNode1);
	}
	else {
		mVa = 0;
	}
	if (mNode2 >= 0) {
		mVb = system.getRealFromLeftSideVector(mNode2);
	}
	else {
		mVb = 0;
	}
	if (mNode3 >= 0) {
		mVc = system.getRealFromLeftSideVector(mNode3);
	}
	else {
		mVc = 0;
	}
}


Matrix SynchronGeneratorEMTFnP::parkTransform2(Real theta, Real a, Real b, Real c) {

	Matrix dq0vector(3, 1);

	// Park transform according to Kundur
	Real d, q;

	d = 2. / 3. * cos(theta) * a + 2. / 3. * cos(theta - 2. * M_PI / 3.)*b + 2. / 3. * cos(theta + 2. * M_PI / 3.)*c;
	q = -2. / 3. * sin(theta)*a - 2. / 3. * sin(theta - 2. * M_PI / 3.)*b - 2. / 3. * sin(theta + 2. * M_PI / 3.)*c;

	//Real zero;
	//zero = 1. / 3. * a, 1. / 3. * b, 1. / 3. * c;

	dq0vector << d,
		q,
		0;

	return dq0vector;
}


Matrix SynchronGeneratorEMTFnP::inverseParkTransform2(Real theta, Real d, Real q, Real zero) {

	Matrix abcVector(3, 1);

	Real a, b, c;

	// Park transform according to Kundur
	a = cos(theta)*d - sin(theta)*q + 1.*zero;
	b = cos(theta - 2. * M_PI / 3.)*d - sin(theta - 2. * M_PI / 3.)*q + 1.*zero;
	c = cos(theta + 2. * M_PI / 3.)*d - sin(theta + 2. * M_PI / 3.)*q + 1.*zero;

	abcVector << a,
		b,
		c;

	return abcVector;
}