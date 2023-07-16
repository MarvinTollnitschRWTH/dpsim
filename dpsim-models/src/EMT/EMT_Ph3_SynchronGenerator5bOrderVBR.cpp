/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_SynchronGenerator5bOrderVBR.h>

using namespace CPS;

EMT::Ph3::SynchronGenerator5bOrderVBR::SynchronGenerator5bOrderVBR
    (const String & uid, const String & name, Logger::Level logLevel)
	: ReducedOrderSynchronGeneratorVBR(uid, name, logLevel),
	mEdq0_t(mAttributes->create<Matrix>("Edq0_t")),
	mEdq0_s(mAttributes->create<Matrix>("Edq0_s")) {

	//
	mSGOrder = SGOrder::SG5bOrder;
	
	// model specific variables
	**mEdq0_t = Matrix::Zero(3,1);
	**mEdq0_s = Matrix::Zero(3,1);
	mEh_t = Matrix::Zero(3,1);
	mEh_s = Matrix::Zero(3,1);
}

EMT::Ph3::SynchronGenerator5bOrderVBR::SynchronGenerator5bOrderVBR
	(const String & name, Logger::Level logLevel)
	: SynchronGenerator5bOrderVBR(name, name, logLevel) {
}

void EMT::Ph3::SynchronGenerator5bOrderVBR::specificInitialization() {
	// initial voltage behind the transient reactance in the dq reference frame
	(**mEdq0_t)(0,0) = 0.0;
	(**mEdq0_t)(1,0) = (1 - mTaa / mTd0_t) * **mEf - (mLd - mLd_t - mYd) * (**mIdq0)(0,0);

	// initial dq behind the subtransient reactance in the dq reference frame
	(**mEdq0_s)(0,0) = (**mVdq0)(0,0) - mLq_s * (**mIdq0)(1,0);
	(**mEdq0_s)(1,0) = (**mVdq0)(1,0) + mLd_s * (**mIdq0)(0,0);

	mSLog->info(
		"\n--- Model specific initialization  ---"
		"\nSG model: 5th order type 2"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\nInitial Ed_s (per unit): {:f}"
		"\nInitial Eq_s (per unit): {:f}"
		"\n--- Model specific initialization finished ---",

		(**mEdq0_t)(0,0),
		(**mEdq0_t)(1,0),
		(**mEdq0_s)(0,0),
		(**mEdq0_s)(1,0)
	);
	mSLog->flush();
}

void EMT::Ph3::SynchronGenerator5bOrderVBR::stepInPerUnit() {

	if (mSimTime>0.0) {
		// calculate Edq_t at t=k
		(**mEdq0_t)(0,0) = 0.0;
		(**mEdq0_t)(1,0) = mAq_t * (**mIdq0)(0,0) + mEh_t(1,0);
		(**mEdq0_t)(2,0) = 0.0;

		// calculate Edq_s at t=k
		(**mEdq0_s)(0,0) = (**mVdq0)(0,0) - mLq_s * (**mIdq0)(1,0);
		(**mEdq0_s)(1,0) = (**mVdq0)(1,0) + mLd_s * (**mIdq0)(0,0);
	}

	// get transformation matrix
	mAbcToDq0 = get_parkTransformMatrix();
	mDq0ToAbc = get_inverseParkTransformMatrix();

	// calculate resistance matrix at t=k+1
	calculateResistanceMatrix();

	// calculate history term behind the transient reactance
	mEh_t(0,0) = 0.0;
	mEh_t(1,0) = mAq_t * (**mIdq0)(0,0) + mBq_t * (**mEdq0_t)(1,0) + mDq_t * (**mEf) + mDq_t * mEf_prev;
	mEh_t(2,0) = 0.0;

	// calculate history term behind the subtransient reactance
	mEh_s(0,0) = mAd_s * (**mIdq0)(1,0) + mCd_s * (**mEdq0_s)(0,0);
	mEh_s(1,0) = mAq_s * (**mIdq0)(0,0) + mBq_s * (**mEdq0_t)(1,0) + mCq_s * (**mEdq0_s)(1,0) + mDq_s * (**mEf) + mDq_s * mEf_prev;
	mEh_s(2,0) = 0.0;

	// convert Edq_s into the abc reference frame
	mEvbr = mDq0ToAbc * mEh_s * mBase_V;
}

