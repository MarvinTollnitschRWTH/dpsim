// SPDX-FileCopyrightText: 2025 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

void EMT_Ph1_C1R1Vs_RC() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_C1R1Vs_RC";

  // Nodes
  auto n1 = SimNode<Real>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Real>::make("n2", PhaseType::Single);

  // Components

  auto c = EMT::Ph1::Capacitor::make("c_rc");
  c->setParameters(0.01);

  auto r = EMT::Ph1::Resistor::make("r_rc");
  r->setParameters(10.0);

  auto vs = EMT::Ph1::VoltageSource::make("vs_rc");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode<Real>::List{SimNode<Real>::GND, n1});

  c->connect(SimNode<Real>::List{n1, n2});
  r->connect(SimNode<Real>::List{n2, SimNode<Real>::GND});

  // Define system topology
  auto sys =
      SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, c, r});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_c_rc", c->attribute("v_intf"));
  logger->logAttribute("i_c_rc", c->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_Ph1_C1R1Vs_generalizedSSN() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_C1R1Vs_generalizedSSN";

  // Nodes
  auto n1 = SimNode<Real>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Real>::make("n2", PhaseType::Single);

  // Components

  auto c = EMT::Ph1::SSNTypeI2T::make("c_genSSN");
  //du_c/dt = i_c/c , y = u_c --> u = i_c, x = u_c, A = 0, B = 1/c, C = 1, D = 0
  double c_param = 0.01;
  Matrix A = Matrix::Zero(1, 1);
  Matrix B = Matrix::Zero(1, 1);
  B(0, 0) = (1 / c_param);
  Matrix C = Matrix::Zero(1, 1);
  C(0, 0) = 1;
  Matrix D = Matrix::Zero(1, 1);
  c->setParameters(A, B, C, D);

  auto r = EMT::Ph1::Resistor::make("r_genSSN");
  r->setParameters(10.0);

  auto vs = EMT::Ph1::VoltageSource::make("vs_genSSN");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode<Real>::List{SimNode<Real>::GND, n1});

  c->connect(SimNode<Real>::List{n1, n2});
  r->connect(SimNode<Real>::List{n2, SimNode<Real>::GND});

  // Define system topology
  auto sys =
      SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, c, r});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_c_genSSN", c->attribute("v_intf"));
  logger->logAttribute("i_c_genSSN", c->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_Ph1_L1R1Vs_RC() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_L1R1Vs_RC";

  // Nodes
  auto n1 = SimNode<Real>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Real>::make("n2", PhaseType::Single);

  // Components

  auto l = EMT::Ph1::Inductor::make("l_rc");
  l->setParameters(0.01);

  auto r = EMT::Ph1::Resistor::make("r_rc");
  r->setParameters(10.0);

  auto vs = EMT::Ph1::VoltageSource::make("vs_rc");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode<Real>::List{SimNode<Real>::GND, n1});

  l->connect(SimNode<Real>::List{n1, n2});
  r->connect(SimNode<Real>::List{n2, SimNode<Real>::GND});

  // Define system topology
  auto sys =
      SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, l, r});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_l_rc", l->attribute("v_intf"));
  logger->logAttribute("i_l_rc", l->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_Ph1_L1R1Vs_generalizedSSN() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_L1R1Vs_generalizedSSN";

  // Nodes
  auto n1 = SimNode<Real>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Real>::make("n2", PhaseType::Single);

  // Components

  auto l = EMT::Ph1::SSNTypeV2T::make("l_genSSN");
  //di_L/dt = v_L/L , y = i_L --> u = v_L, x = i_L, A = 0, B = 1/L, C = 1, D = 0
  double l_param = 0.01;
  Matrix A = Matrix::Zero(1, 1);
  Matrix B = Matrix::Zero(1, 1);
  B(0, 0) = (1 / l_param);
  Matrix C = Matrix::Zero(1, 1);
  C(0, 0) = 1;
  Matrix D = Matrix::Zero(1, 1);
  l->setParameters(A, B, C, D);

  auto r = EMT::Ph1::Resistor::make("R1");
  r->setParameters(10.0);

  auto vs = EMT::Ph1::VoltageSource::make("vs");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode<Real>::List{SimNode<Real>::GND, n1});

  l->connect(SimNode<Real>::List{n1, n2});
  r->connect(SimNode<Real>::List{n2, SimNode<Real>::GND});

  // Define system topology
  auto sys =
      SystemTopology(50, SystemNodeList{n1, n2}, SystemComponentList{vs, l, r});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_l_genSSN", l->attribute("v_intf"));
  logger->logAttribute("i_l_genSSN", l->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_Ph1_RLCVs_RC() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_RLCVs_RC";

  // Nodes
  auto n1 = SimNode<Real>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Real>::make("n2", PhaseType::Single);
  auto n3 = SimNode<Real>::make("n3", PhaseType::Single);

  // Components

  auto r = EMT::Ph1::Resistor::make("r_rc");
  r->setParameters(10.0);

  auto l = EMT::Ph1::Inductor::make("l_rc");
  l->setParameters(0.01);

  auto c = EMT::Ph1::Capacitor::make("c_rc");
  c->setParameters(0.002);

  auto vs = EMT::Ph1::VoltageSource::make("vs_rc");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode<Real>::List{SimNode<Real>::GND, n1});

  r->connect(SimNode<Real>::List{n1, n2});
  l->connect(SimNode<Real>::List{n2, n3});
  c->connect(SimNode<Real>::List{n3, SimNode<Real>::GND});

  // Define system topology
  auto sys = SystemTopology(50, SystemNodeList{n1, n2, n3},
                            SystemComponentList{vs, r, l, c});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_l_rlc_rc", l->attribute("v_intf"));
  logger->logAttribute("i_l_rlc_rc", l->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_Ph1_RLCVs_explicitSSN() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_RLCVs_explicitSSN";

  // Nodes
  auto n1 = SimNode<Real>::make("n1", PhaseType::Single);

  // Components

  auto rlc = EMT::Ph1::SSN::Full_Serial_RLC::make("rlc");
  double r_param = 10.0;
  double l_param = 0.01;
  double c_param = 0.002;

  rlc->setParameters(r_param, l_param, c_param);

  auto vs = EMT::Ph1::VoltageSource::make("vs");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode<Real>::List{SimNode<Real>::GND, n1});

  rlc->connect(SimNode<Real>::List{n1, SimNode<Real>::GND});

  // Define system topology
  auto sys =
      SystemTopology(50, SystemNodeList{n1}, SystemComponentList{vs, rlc});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_rlc_explSSN", rlc->attribute("v_intf"));
  logger->logAttribute("i_rlc_explSSN", rlc->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_Ph1_RLCVs_generalizedSSN() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_RLCVs_generalizedSSN";

  // Nodes
  auto n1 = SimNode<Real>::make("n1", PhaseType::Single);

  // Components

  auto rlc = EMT::Ph1::SSNTypeV2T::make("rlc_genSSN");

  double r = 10.0;
  double l = 0.01;
  double c = 0.002;

  Matrix A = Matrix::Zero(2, 2);
  A(0, 0) = 0;
  A(0, 1) = 1. / c;
  A(1, 0) = -1. / l;
  A(1, 1) = -r / l;
  Matrix B = Matrix::Zero(2, 1);
  B(0, 0) = 0;
  B(1, 0) = 1. / l;
  Matrix C = Matrix::Zero(1, 2);
  C(0, 0) = 0;
  C(0, 1) = 1;
  Matrix D = Matrix::Zero(1, 1);
  rlc->setParameters(A, B, C, D);

  auto vs = EMT::Ph1::VoltageSource::make("vs");
  vs->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))), 50.0);

  // Topology
  vs->connect(SimNode<Real>::List{SimNode<Real>::GND, n1});

  rlc->connect(SimNode<Real>::List{n1, SimNode<Real>::GND});

  // Define system topology
  auto sys =
      SystemTopology(50, SystemNodeList{n1}, SystemComponentList{vs, rlc});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_rlc_genSSN", rlc->attribute("v_intf"));
  logger->logAttribute("i_rlc_genSSN", rlc->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}

void EMT_Ph1_RLCVs_RC_PFinit() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_RLCVs_RC_withInit";

  // ----- POWERFLOW FOR INITIALIZATION ---
  Real timeStepPF = finalTime;
  Real finalTimePF = finalTime + timeStepPF;
  String simNamePF = simName + "_PF";

  // Nodes
  auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
  auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);

  // Components

  auto rPF = SP::Ph1::Resistor::make("r_rc");
  rPF->setParameters(10.0);

  auto lPF = SP::Ph1::Inductor::make("l_rc");
  lPF->setParameters(0.01);

  auto cPF = SP::Ph1::Capacitor::make("c_rc");
  cPF->setParameters(0.002);

  auto vsPF = SP::Ph1::VoltageSource::make("vs_rc");
  vsPF->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))),
                      50.0);

  // Topology
  vsPF->connect(SimNode<Complex>::List{SimNode<Complex>::GND, n1PF});

  rPF->connect(SimNode<Complex>::List{n1PF, n2PF});
  lPF->connect(SimNode<Complex>::List{n2PF, n3PF});
  cPF->connect(SimNode<Complex>::List{n3PF, SimNode<Complex>::GND});

  // Define system topology
  auto sysPF = SystemTopology(50, SystemNodeList{n1PF, n2PF, n3PF},
                              SystemComponentList{vsPF, rPF, lPF, cPF});

  // Logging
  Logger::setLogDir("logs/" + simNamePF);
  auto loggerPF = DataLogger::make(simNamePF);
  loggerPF->logAttribute("v1", n1PF->attribute("v"));
  loggerPF->logAttribute("v2", n2PF->attribute("v"));
  loggerPF->logAttribute("v3", n3PF->attribute("v"));

  Simulation simPF(simNamePF, Logger::Level::info);
  simPF.setSystem(sysPF);
  simPF.addLogger(loggerPF);
  simPF.setDomain(Domain::SP);
  simPF.setSolverType(Solver::Type::NRP);
  simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  simPF.doInitFromNodesAndTerminals(false);
  simPF.setTimeStep(timeStepPF);
  simPF.setFinalTime(finalTimePF);
  simPF.run();

  // ----- DYNAMIC SIMULATION -----
  // Define simulation scenario
  Real timeStepEMT = timeStep;
  Real finalTimeEMT = finalTime + timeStepEMT;

  // Nodes
  auto n1EMT = SimNode<Real>::make("n1", PhaseType::Single);
  auto n2EMT = SimNode<Real>::make("n2", PhaseType::Single);
  auto n3EMT = SimNode<Real>::make("n3", PhaseType::Single);

  // Components

  auto rEMT = EMT::Ph1::Resistor::make("r_rc");
  rEMT->setParameters(10.0);

  auto lEMT = EMT::Ph1::Inductor::make("l_rc");
  lEMT->setParameters(0.01);

  auto cEMT = EMT::Ph1::Capacitor::make("c_rc");
  cEMT->setParameters(0.002);

  auto vsEMT = EMT::Ph1::VoltageSource::make("vs_rc");
  vsEMT->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))),
                       50.0);

  // Topology
  vsEMT->connect(SimNode<Real>::List{SimNode<Real>::GND, n1EMT});

  rEMT->connect(SimNode<Real>::List{n1EMT, n2EMT});
  lEMT->connect(SimNode<Real>::List{n2EMT, n3EMT});
  cEMT->connect(SimNode<Real>::List{n3EMT, SimNode<Real>::GND});

  // Define system topology
  auto sysEMT = SystemTopology(50, SystemNodeList{n1EMT, n2EMT, n3EMT},
                               SystemComponentList{vsEMT, rEMT, lEMT, cEMT});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto loggerEMT = DataLogger::make(simName);
  loggerEMT->logAttribute("v_l_rlc_rc", lEMT->attribute("v_intf"));
  loggerEMT->logAttribute("i_l_rlc_rc", lEMT->attribute("i_intf"));

  //Initialization of dynamic topology
  sysEMT.initWithPowerflow(sysPF, Domain::EMT);

  //Simulation
  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sysEMT);
  sim.addLogger(loggerEMT);
  sim.setDomain(Domain::EMT);
  sim.setTimeStep(timeStepEMT);
  sim.setFinalTime(finalTimeEMT);
  sim.run();
}

void EMT_Ph1_RLCVs_generalizedSSN_PFinit() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_General2TerminalSSN_RLCVs_generalizedSSN_withInit";

  // ----- POWERFLOW FOR INITIALIZATION -----
  Real timeStepPF = finalTime;
  Real finalTimePF = finalTime + timeStepPF;
  String simNamePF = simName + "_PF";

  // Nodes
  auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);

  // Components

  auto rlcPF = SP::Ph1::SSNTypeV2T::make("rlc_genSSN");

  double r = 10.0;
  double l = 0.01;
  double c = 0.002;

  Matrix A = Matrix::Zero(2, 2);
  A(0, 0) = 0;
  A(0, 1) = 1. / c;
  A(1, 0) = -1. / l;
  A(1, 1) = -r / l;
  Matrix B = Matrix::Zero(2, 1);
  B(0, 0) = 0;
  B(1, 0) = 1. / l;
  Matrix C = Matrix::Zero(1, 2);
  C(0, 0) = 0;
  C(0, 1) = 1;
  Matrix D = Matrix::Zero(1, 1);
  rlcPF->setParameters(A, B, C, D);

  auto vsPF = SP::Ph1::VoltageSource::make("vs");
  vsPF->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))),
                      50.0);

  // Topology
  vsPF->connect(SimNode<Complex>::List{SimNode<Complex>::GND, n1PF});

  rlcPF->connect(SimNode<Complex>::List{n1PF, SimNode<Complex>::GND});

  // Define system topology
  auto sysPF = SystemTopology(50, SystemNodeList{n1PF},
                              SystemComponentList{vsPF, rlcPF});

  // Logging
  Logger::setLogDir("logs/" + simNamePF);
  auto loggerPF = DataLogger::make(simNamePF);
  loggerPF->logAttribute("v1", n1PF->attribute("v"));

  Simulation simPF(simNamePF, Logger::Level::info);
  simPF.setSystem(sysPF);
  simPF.addLogger(loggerPF);
  simPF.setDomain(Domain::SP);
  simPF.setSolverType(Solver::Type::NRP);
  simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  simPF.doInitFromNodesAndTerminals(false);
  simPF.setTimeStep(timeStepPF);
  simPF.setFinalTime(finalTimePF);
  simPF.run();

  // ----- DYNAMIC SIMULATION -----
  // Define simulation scenario
  Real timeStepEMT = timeStep;
  Real finalTimeEMT = finalTime + timeStepEMT;

  // Nodes
  auto n1EMT = SimNode<Real>::make("n1", PhaseType::Single);

  // Components

  auto rlcEMT = EMT::Ph1::SSNTypeV2T::make("rlc_genSSN");

  A = Matrix::Zero(2, 2);
  A(0, 0) = 0;
  A(0, 1) = 1. / c;
  A(1, 0) = -1. / l;
  A(1, 1) = -r / l;
  B = Matrix::Zero(2, 1);
  B(0, 0) = 0;
  B(1, 0) = 1. / l;
  C = Matrix::Zero(1, 2);
  C(0, 0) = 0;
  C(0, 1) = 1;
  D = Matrix::Zero(1, 1);
  rlcEMT->setParameters(A, B, C, D);

  auto vsEMT = EMT::Ph1::VoltageSource::make("vs");
  vsEMT->setParameters((CPS::Math::polar(1.0, CPS::Math::degToRad(-90.0))),
                       50.0);

  // Topology
  vsEMT->connect(SimNode<Real>::List{SimNode<Real>::GND, n1EMT});

  rlcEMT->connect(SimNode<Real>::List{n1EMT, SimNode<Real>::GND});

  // Define system topology
  auto sysEMT = SystemTopology(50, SystemNodeList{n1EMT},
                               SystemComponentList{vsEMT, rlcEMT});

  // Logging
  Logger::setLogDir("logs/" + simName);
  auto loggerEMT = DataLogger::make(simName);
  loggerEMT->logAttribute("v_rlc_genSSN", rlcEMT->attribute("v_intf"));
  loggerEMT->logAttribute("i_rlc_genSSN", rlcEMT->attribute("i_intf"));

  //Initialization of dynamic topology
  sysEMT.initWithPowerflow(sysPF, Domain::EMT);

  //Simulation
  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sysEMT);
  sim.addLogger(loggerEMT);
  sim.setDomain(Domain::EMT);
  sim.doInitFromNodesAndTerminals(true);
  sim.setTimeStep(timeStepEMT);
  sim.setFinalTime(finalTimeEMT);
  sim.run();
}

int main(int argc, char *argv[]) {
  EMT_Ph1_C1R1Vs_RC();
  EMT_Ph1_C1R1Vs_generalizedSSN();

  EMT_Ph1_L1R1Vs_RC();
  EMT_Ph1_L1R1Vs_generalizedSSN();

  EMT_Ph1_RLCVs_RC();
  EMT_Ph1_RLCVs_explicitSSN();
  EMT_Ph1_RLCVs_generalizedSSN();

  EMT_Ph1_RLCVs_RC_PFinit();
  EMT_Ph1_RLCVs_generalizedSSN_PFinit();
}
