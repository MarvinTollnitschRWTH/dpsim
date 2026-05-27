// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNATearInterface.h>

namespace CPS {
namespace SP {
namespace Ph1 {
/// \brief SSNTypeV2T
/// Model for a one phase, two terminal V-type SSN component which can be represented using
/// a state space equation system
/// x' = A * x + B * u
/// y = C * x + D * u
/// with x: state vector, y: output vector, u: input vector,
/// where u represents external voltage (mIntfVoltage),
/// y represents external current (mIntfCurrent),
/// x represents any component states.
class SSNTypeV2T : public MNASimPowerComp<Complex>,
                   public SharedFactory<SSNTypeV2T>,
                   public MNATearInterface {
private:
  void ssnUpdateState();
  void setSSNMatricesToZero();

protected:
  MatrixComp mX;
  MatrixComp mU;
  MatrixComp mUOld;
  MatrixComp mW;
  MatrixComp mYHist;
  Real mOmega;
  /// susceptance [S]
  Complex mSusceptance;

public:
  const CPS::Attribute<MatrixComp>::Ptr mA;
  const CPS::Attribute<MatrixComp>::Ptr mB;
  const CPS::Attribute<MatrixComp>::Ptr mC;
  const CPS::Attribute<MatrixComp>::Ptr mD;

  /// Defines UID, name, component parameters and logging level
  SSNTypeV2T(String uid, String name,
             Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  SSNTypeV2T(String name, Logger::Level logLevel = Logger::Level::off)
      : SSNTypeV2T(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  void manualInit(Matrix initialState, Matrix initialInput,
                  Matrix initialOldInput, Real initCurrent, Real initVoltage);

  // #### General ####
  void setParameters(const MatrixComp A, const MatrixComp B, const MatrixComp C,
                     const MatrixComp D);
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;

  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;
  void mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix);
};
} // namespace Ph1
} // namespace SP
} // namespace CPS
