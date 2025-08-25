#pragma once
#include "scip_exception.h"

class AirportGraph;

// constraints:
void create_demand_constraints(AirportGraph *airportgraph, SCIP *scip);
void create_elec_cons(AirportGraph *airportgraph, SCIP *scip);
void create_budget_constraint(AirportGraph *airportgraph, SCIP *scip, double budget);
void create_cost_optimal_path_constraints(AirportGraph *airportgraph, SCIP *scip);