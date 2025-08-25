#include "model_init.h"
#include <scip/scipdefplugins.h>
#include "../../seaport_class.h"
#include "../../graph/airport_graph.h"
#include "../pricer/path_pricer.h"
#include "../constraint_handlers/costoptimalpath_cons.h"

SCIP *create_basic_model(std::string name, AirportGraph *airportgraph)
{
    SCIP *scip;
    SCIP_CALL_EXC(SCIPcreate(&scip));
    SCIP_CALL_EXC(SCIPincludeDefaultPlugins(scip));
    SCIP_CALL_EXC(SCIPcreateProbBasic(scip, name.c_str()));
    SCIP_CALL_EXC(SCIPsetRealParam(scip, "limits/time", 7200));

    return scip;
}

void include_path_pricer(PathPricer *pathpricer, AirportGraph *airportgraph, SCIP *scip)
{
    SCIP_CALL_EXC(SCIPincludeObjPricer(scip, pathpricer, true));
    SCIP_CALL_EXC(SCIPactivatePricer(scip, SCIPfindPricer(scip, "pathpricer")));
}

void include_optpath_conshdl(ConshdlrOptPaths *conshdlr, SCIP *scip)
{
    SCIP_CALL_EXC(SCIPincludeObjConshdlr(scip, conshdlr, true));
}

void release_all_vars(AirportGraph *airportgraph, SCIP *scip)
{
    auto g = airportgraph->graph;
    for (int v = 0; v < airportgraph->number_of_airports; v++)
    {
        SCIP_CALL_EXC(SCIPreleaseVar(scip, &(airportgraph->map_vertex_to_variable[airportgraph->node_vector[v]])));
        for (int w = v + 1; w < airportgraph->number_of_airports; w++)
        {
            auto paths = airportgraph->get_path_variables_for_index_pair(v, w);
            for (auto path : paths)
            {
                SCIPreleaseVar(scip, &path);
            }
            auto var = airportgraph->get_bilinear_variable_for_index_pair(v, w);
            if (var != NULL)
            {
                SCIPreleaseVar(scip, &var);
            }
        }
    }
}
