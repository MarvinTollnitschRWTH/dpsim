/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 ******************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_QuadraticResistor.h>

using namespace CPS;

EMT::Ph1::QuadraticResistor::QuadraticResistor(String uid, String name,
                                             Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, false, false, logLevel),
      mR(mAttributes->create<Real>("I_S")){
  mPhaseType = PhaseType::Single;
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
}

void EMT::Ph1::QuadraticResistor::setParameters(Real R) {
  **mR = R;
}

SimPowerComp<Real>::Ptr EMT::Ph1::QuadraticResistor::clone(String name) {
  auto copy = QuadraticResistor::make(name, mLogLevel);
  copy->setParameters(**mR);
  return copy;
}

void EMT::Ph1::QuadraticResistor::initializeFromNodesAndTerminals(
    Real frequency) {

  // IntfVoltage initialization for each phase
  MatrixComp vInitABC = Matrix::Zero(1, 1);
  vInitABC(0, 0) = initialSingleVoltage(0) -
                   initialSingleVoltage(1);
  (**mIntfVoltage)(0, 0) = vInitABC(0, 0).real();

  (**mIntfCurrent)(0, 0) =
      (1./(**mR)) * (**mIntfVoltage)(0, 0) * (**mIntfVoltage)(0, 0);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:f}"
                     "\nCurrent: {:f}"
                     "\nTerminal 0 voltage: {:f}"
                     "\nTerminal 1 voltage: {:f}"
                     "\n--- Initialization from powerflow finished ---",
                     (**mIntfVoltage)(0, 0), (**mIntfCurrent)(0, 0),
                     initialSingleVoltage(0).real(),
                     initialSingleVoltage(1).real());
}

void EMT::Ph1::QuadraticResistor::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {

  updateMatrixNodeIndices();
  **mNonlinearFunctionStamp = Matrix::Zero(leftVector->get().rows(), 1);
  calculateNonlinearFunctionResult(**leftVector);
  updateJacobian();

  mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void EMT::Ph1::QuadraticResistor::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  // Set diagonal entries
  if (terminalNotGrounded(0)) {
    // set upper left block, 3x3 entries
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0),
                             matrixNodeIndex(0, 0), Jacobian(0, 0));
  }
  if (terminalNotGrounded(1)) {
    // set buttom right block, 3x3 entries
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0),
                             matrixNodeIndex(1, 0), Jacobian(0, 0));
  }
  // Set off diagonal blocks, 2x3x3 entries
  if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0),
                             matrixNodeIndex(1, 0), -Jacobian(0, 0));

    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0),
                             matrixNodeIndex(0, 0), -Jacobian(0, 0));
  }
}

void EMT::Ph1::QuadraticResistor::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

// void EMT::Ph1::QuadraticResistor::mnaCompPreStep(Real time, Int timeStepCount) {
//   mnaCompApplyRightSideVectorStamp(**mRightVector);
// }

void EMT::Ph1::QuadraticResistor::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaUpdateVoltage(**leftVector);
  mnaUpdateCurrent(**leftVector);
}

void EMT::Ph1::QuadraticResistor::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  // v0 - v1, anode to cathode
  (**mIntfVoltage)(0,0) = 0.;
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  }
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
  }
}

void EMT::Ph1::QuadraticResistor::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) =
      (1./(**mR)) * (**mIntfVoltage)(0, 0) * (**mIntfVoltage)(0, 0);
      if((**mIntfVoltage)(0, 0) < 0.0) {
        (**mIntfCurrent)(0, 0) = - (**mIntfCurrent)(0, 0);
      } 
}

void EMT::Ph1::QuadraticResistor::iterationUpdate(const Matrix &leftVector) {
  //Update phase voltages
  (**mIntfVoltage)(0, 0) = 0.;
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  }
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
  }
  //std::cout << leftVector << std::endl;
  //std::cout << **mIntfVoltage << std::endl;
  updateJacobian();
}

void EMT::Ph1::QuadraticResistor::calculateNonlinearFunctionResult(const Matrix &leftVector) {

  Real IntfV = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0)) -  Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
  Real nonLinearResult = (1./(**mR)) * IntfV * IntfV;
  if(IntfV < 0.0) {
    nonLinearResult = - nonLinearResult;
  } 

  if (terminalNotGrounded(1)) {
    Math::setVectorElement(**mNonlinearFunctionStamp, matrixNodeIndex(1, 0),
                           nonLinearResult);
  }
  if (terminalNotGrounded(0)) {
    Math::setVectorElement(**mNonlinearFunctionStamp, matrixNodeIndex(0, 0),
                           -nonLinearResult);
  }
}

void EMT::Ph1::QuadraticResistor::updateJacobian() {
  Jacobian(0, 0) =
      2.0 * (1./(**mR)) * (**mIntfVoltage)(0, 0);
  if((**mIntfVoltage)(0, 0) < 0.0) {
    Jacobian(0, 0) = - Jacobian(0, 0);
  }
      //std::cout << "IntfV:" << (**mIntfVoltage)(0,0) << std::endl << std::endl;
      //std::cout << "IntfI:" << (**mIntfCurrent)(0,0) << std::endl << std::endl;
      //std::cout << "Jacobian: " << Jacobian(0,0) << std::endl << std::endl;
    
}