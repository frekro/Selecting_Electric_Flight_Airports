#pragma once
#include "scip_exception.h"
#include <vector>

class AirportGraph;

// variables:
void create_airport_variables(AirportGraph *airportgraph, SCIP *scip, std::vector<int> bigMs);
void create_bilinear_variables(AirportGraph *airportgraph, SCIP *scip);
