/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/MNANonlinearVariableCompInterface.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
/// Nonlinear Resistor: I = R*(V^2)
class QuadraticResistor : public MNASimPowerComp<Real>,
                         public MNANonlinearVariableCompInterface,
                         public SharedFactory<QuadraticResistor> {
protected:
  Matrix Jacobian = Matrix::Zero(1, 1);

public:
  //Resistive constant Factor
  const CPS::Attribute<Real>::Ptr mR;

  /// Defines UID, name, component parameters and logging level
  QuadraticResistor(String uid, String name,
                   Logger::Level logLevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  QuadraticResistor(String name, Logger::Level logLevel = Logger::Level::off)
      : QuadraticResistor(name, name, logLevel) {}

  // #### General ####
  ///
  SimPowerComp<Real>::Ptr clone(String name) override;
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  void setParameters(Real R);

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftSideVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override {
  } //No right side vector stamps
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// MNA pre and post step operations
  // void mnaCompPreStep(Real time,
  //                     Int timeStepCount) override; //No right side vector stamps
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  /// add MNA pre and post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

  virtual void iterationUpdate(const Matrix &leftVector) override;

  virtual bool hasParameterChanged() override { return true; }

  void calculateNonlinearFunctionResult(const Matrix &leftVector) override;

  virtual void updateJacobian() override;
};
} // namespace Ph1
} // namespace EMT
} // namespace CPS
