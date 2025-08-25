#include "seaport_class.h"
#include "scip_instance/model/model_init.h"
#include "scip_instance/model/variables.h"
#include "scip_instance/model/constraints.h"
#include "scip_instance/pricer/path_pricer_rc.h"
#include "../global_parameter.h"
#include "../dump.h"

#include "scip_instance/constraint_handlers/costoptimalpath_cons.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

SeaportProgram::SeaportProgram(Input_SeaportProgram input) : airportgraph(input.airportgraph),
                                                             budget(input.budget / constants::SCALING_FACTOR_EMISSION)
{
    bigM = 2 * (input.airportgraph->number_of_airports) * (input.airportgraph->number_of_airports);
    bigMs = std::vector<int>(airportgraph->number_of_airports, bigM);

    model = create_basic_model("seaport_location", airportgraph);

    // pathpricer = std::make_unique<PathPricerNormal>(airportgraph, model, "pathpricer");
    // include_path_pricer(pathpricer.get(), airportgraph, model);

    pathpricer = std::make_unique<PathPricerWRC>(airportgraph, model, "pathpricer");
    include_path_pricer(pathpricer.get(), airportgraph, model);

    optpath_conshdlr = std::make_unique<ConshdlrOptPaths>(model, airportgraph);
    include_optpath_conshdl(optpath_conshdlr.get(), model);

    // constraints:
    create_elec_cons(airportgraph, model);
    create_demand_constraints(airportgraph, model);
    create_budget_constraint(airportgraph, model, budget);
    create_cost_optimal_path_constraints(airportgraph, model);

    // //  variables:
    create_airport_variables(airportgraph, model, bigMs);
    create_bilinear_variables(airportgraph, model);
}

SeaportProgram::~SeaportProgram() {
    // release_all_vars(airportgraph, model);
};