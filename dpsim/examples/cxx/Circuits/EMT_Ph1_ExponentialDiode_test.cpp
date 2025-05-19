#include "../Examples.h"
#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT;

void EMT_Ph1_ExponentialDiode() {
  // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_ExponentialDiode_test";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  Real initialVoltage_n1 = 1.0;
  Real initialVoltage_n2 = 0.4;
  auto n1 = SimNode::make("n1", PhaseType::Single);
  auto n2 = SimNode::make("n2", PhaseType::Single);
  n1->setInitialVoltage(initialVoltage_n1);
  n2->setInitialVoltage(initialVoltage_n2);

  // Components

  auto vs0 = Ph1::VoltageSource::make("vs0", Logger::Level::debug);
  vs0->setParameters(CPS::Complex(1., 0.), 50.0);

  auto load = CPS::EMT::Ph1::Resistor::make("Load", Logger::Level::debug);
  load->setParameters(10.);

  auto expDiode = Ph1::ExponentialDiode::make("ExponentialDiode", Logger::Level::debug);
  expDiode->setParameters(
      1.0e-12, 25.852e-3); //Calling this is optional. If this method call
                        //is omitted, the diode will get the following
                        //values by default:
                        //I_S = 1.0e-12 (A) and V_T = 25.852e-3 (V).

  // Topology
  vs0->connect(SimNode::List{SimNode::GND, n1});

  load->connect(SimNode::List{n2, SimNode::GND});

  expDiode->connect(SimNode::List{n1, n2}); //Connect diode in the
                                                      //forward direction, i.e.
                                                      //from anode (+) to
                                                      //cathode (-)
                                                      //Diode Equation:
  //I_D = (**mI_S) * (expf((**mIntfVoltage)(0, 0) / (**mV_T)) - 1.)

  // Define system topology
  auto sys = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{load, vs0, expDiode});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("V_1", n1->attribute("v"));
  logger->logAttribute("V_2", n2->attribute("v"));
  logger->logAttribute("I_ExponentialDiode", expDiode->attribute("i_intf"));
  logger->logAttribute("V_ExponentialDiode", expDiode->attribute("v_intf"));
  logger->logAttribute("I_Load", load->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::debug);
  sim.doInitFromNodesAndTerminals(true);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setSolverType(Solver::Type::ITERATIVEMNA);
  sim.setDirectLinearSolverConfiguration(DirectLinearSolverConfiguration());
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}


void connectionTest(){
 // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_connection_test";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  //Real initialVoltage_n1 = 1.0;
  //Real initialVoltage_n2 = 0.4;
  auto n1 = SimNode::make("n1", PhaseType::Single);
  auto n2 = SimNode::make("n2", PhaseType::Single);

  // Components

  auto vs0 = Ph1::VoltageSource::make("vs0", Logger::Level::debug);
  vs0->setParameters(CPS::Complex(1., 0.), 50.0);

  auto load1 = CPS::EMT::Ph1::Resistor::make("Load1", Logger::Level::debug);
  load1->setParameters(10.);

  auto load2 = CPS::EMT::Ph1::Resistor::make("Load2", Logger::Level::debug);;
  load2->setParameters(10.);

  vs0->connect(SimNode::List{SimNode::GND, n1});

  load1->connect(SimNode::List{n1, n2});

  load2->connect(SimNode::List{n2, SimNode::GND}); 

  // Define system topology
  auto sys = SystemTopology(50, SystemNodeList{n1, n2},
                            SystemComponentList{load1, vs0, load2});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("I_Load1", load1->attribute("i_intf"));
  logger->logAttribute("I_Load2", load2->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::debug);
  sim.doInitFromNodesAndTerminals(true);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setSolverType(Solver::Type::ITERATIVEMNA);
  sim.setDirectLinearSolverConfiguration(DirectLinearSolverConfiguration());
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}


void vs_diode(){
 // Define simulation scenario
  Real timeStep = 0.0001;
  Real finalTime = 0.1;
  String simName = "EMT_Ph1_vs_diode";
  Logger::setLogDir("logs/" + simName);

  // Nodes
  const Real initialSingleVoltage_v1 = 1.0;
  auto n1 = SimNode::make("n1", PhaseType::Single);
  n1->setInitialVoltage(initialSingleVoltage_v1);

  // Components

  auto vs0 = Ph1::VoltageSource::make("vs0", Logger::Level::debug);
  vs0->setParameters(CPS::Complex(1., 0.), 50.0);

  auto diode = Ph1::ExponentialDiode::make("diode", Logger::Level::debug);

  vs0->connect(SimNode::List{SimNode::GND, n1});

  diode->connect(SimNode::List{n1, SimNode::GND});

  // Define system topology
  auto sys = SystemTopology(50, SystemNodeList{n1},
                            SystemComponentList{vs0, diode});

  // Logging
  auto logger = DataLogger::make(simName);
  logger->logAttribute("diode_V", diode->attribute("v_intf"));
  logger->logAttribute("diode_I", diode->attribute("i_intf"));

  Simulation sim(simName, Logger::Level::debug);
  sim.doInitFromNodesAndTerminals(true);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setSolverType(Solver::Type::ITERATIVEMNA);
  sim.setDirectLinearSolverConfiguration(DirectLinearSolverConfiguration());
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.run();
}


int main() {
  //EMT_Ph1_ExponentialDiode();
  //connectionTest();
  vs_diode();
  return 0;
}