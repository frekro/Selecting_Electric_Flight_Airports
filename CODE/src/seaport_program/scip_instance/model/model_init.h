#pragma once
#include "scip_exception.h"

class PathPricer;
class ConshdlrOptPaths;
class AirportGraph;

// model_init:
SCIP *create_basic_model(std::string name, AirportGraph *airportgraph);
void include_path_pricer(PathPricer *pathpricer, AirportGraph *airportgraph, SCIP *scip);
void include_optpath_conshdl(ConshdlrOptPaths *conshdlr, SCIP *scip);