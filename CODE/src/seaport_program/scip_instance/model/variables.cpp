#include "variables.h"
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include "../../seaport_class.h"
#include "../../graph/airport_graph.h"
#include "pathvar_prob.h"
#include "scip_exception.h"
#include "../../../dump.h"
#include "../../../global_parameter.h"

SCIP_VAR *scip_create_var_int(SCIP *scip, std::ostringstream &namebuf, double lb, double ub, double obj, bool removable)
{
    SCIP_VAR *var;
    SCIP_CALL_EXC(SCIPcreateVar(scip,
                                &var,
                                namebuf.str().c_str(), // name of var
                                lb,                    // lower
                                ub,                    // upper
                                obj,                   // objective value
                                SCIP_VARTYPE_INTEGER,  // vartype
                                TRUE,                  // initial
                                removable,             // removable
                                NULL, NULL, NULL, NULL, NULL));
    SCIP_CALL_EXC(SCIPaddVar(scip, var));
    return (var);
}

SCIP_VAR *scip_create_var_continous(SCIP *scip, std::ostringstream &namebuf, double lb, double ub, double obj, bool removable)
{
    SCIP_VAR *var;
    SCIP_CALL_EXC(SCIPcreateVar(scip,
                                &var,
                                namebuf.str().c_str(),   // name of var
                                lb,                      // lower
                                ub,                      // upper
                                obj,                     // objective value
                                SCIP_VARTYPE_CONTINUOUS, // vartype
                                TRUE,                    // initial
                                removable,               // removable
                                NULL, NULL, NULL, NULL, NULL));
    SCIP_CALL_EXC(SCIPaddVar(scip, var));
    return (var);
}

void create_airport_variables(AirportGraph *airportgraph, SCIP *scip, std::vector<int> bigMs)
{
    Graph::vertex_iterator vi, vi_end;
    auto g = airportgraph->graph;
    SCIP_VAR *var = NULL;
    for (std::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi)
    {
        std::ostringstream namebuf;
        namebuf << "z_" << g[*vi].name;
        SCIP_VAR *var = NULL;
        if (g[*vi].electrifiable)
        {
            auto electrification_cost = g[*vi].electrification_costs * g[graph_bundle].alpha;
            var = scip_create_var_int(scip, namebuf, 0., 1., 0, TRUE);
            auto cons = airportgraph->vertex_to_elec_constraint[airportgraph->get_index_of_vertex(*vi)];
            auto bigM = bigMs[airportgraph->get_index_of_vertex(*vi)];
            SCIPaddCoefLinear(scip, cons, var, -bigM);
            auto cons_budget = airportgraph->budget_cons;
            SCIPaddCoefLinear(scip, cons_budget, var, electrification_cost);
        }
        airportgraph->map_vertex_to_variable[*vi] = var;
        airportgraph->map_variable_to_vertex[var] = *vi;
    }
};

//  Better connection to heuristic;
SCIP_Var *add_path_var(AirportGraph *airportgraph, SCIP *scip, int v, int w, std::ostringstream &namebuf, double cost, double emission, std::vector<int> vertices, std::vector<int> aircrafts, std::set<Graph::vertex_descriptor> elec_vertices)
{
    if (v >= w)
    {
        return nullptr;
    }

    SCIP_Var *path = createPathVar(scip, namebuf.str(), 0., SCIPinfinity(scip), emission, SCIP_VARTYPE_CONTINUOUS, vertices, aircrafts, cost, emission);
    SCIP_CALL_EXC(SCIPaddVar(scip, path));
    if (elec_vertices.size() > 0)
    {
        for (auto vertex : elec_vertices)
        {
            auto elec_cons = airportgraph->vertex_to_elec_constraint[vertex];
            SCIP_CALL_EXC(SCIPaddCoefLinear(scip, elec_cons, path, 1));
        }
    }

    auto demand_cons = airportgraph->get_demand_constraint_for_index_pair(v, w);
    SCIP_CALL_EXC(SCIPaddCoefLinear(scip, demand_cons, path, 1));

    auto conss = airportgraph->get_optimal_path_constraints_for_vertex_pair(v, w);
    for (auto cons : conss)
    {
        SCIPaddCoefLinear(scip, cons, path, cost);
    }
    airportgraph->insert_path_variable_for_index_pair(v, w, path);

    return path;
}

SCIP_VAR *add_one_bilinear_var_with_mccormick_constraints(AirportGraph *airportgraph, SCIP *scip, int index_v, int index_w)
{
    assert(index_v < index_w);
    std::ostringstream namebuf;
    namebuf << "w_" << airportgraph->get_airport_name_by_index(index_v) << "_" << airportgraph->get_airport_name_by_index(index_w);
    auto w_vw = scip_create_var_continous(scip, namebuf, 0., 1., 0., FALSE);

    airportgraph->index_pair_biliniar_variable[index_type{index_v, index_w}] = w_vw;

    auto z_v = airportgraph->map_vertex_to_variable[airportgraph->node_vector[index_v]];
    auto z_w = airportgraph->map_vertex_to_variable[airportgraph->node_vector[index_w]];

    // w_vw > z_w + z_v -1:
    SCIP_Cons *cons1;
    std::ostringstream consname_1;
    consname_1 << "MC_cons1_" << airportgraph->get_airport_name_by_index(index_v) << "_" << airportgraph->get_airport_name_by_index(index_w);
    SCIP_CALL_EXC(SCIPcreateConsBasicLinear(scip, &cons1, consname_1.str().c_str(), 0, NULL, NULL, -SCIPinfinity(scip), 1));
    SCIP_CALL_EXC(SCIPaddCoefLinear(scip, cons1, z_v, 1));
    SCIP_CALL_EXC(SCIPaddCoefLinear(scip, cons1, z_w, 1));
    SCIP_CALL_EXC(SCIPaddCoefLinear(scip, cons1, w_vw, -1));
    SCIPaddCons(scip, cons1);
    SCIPreleaseCons(scip, &cons1);
    // w_vw < z_v
    SCIP_Cons *cons2;
    std::ostringstream consname_2;
    consname_2 << "MC_cons2_" << airportgraph->get_airport_name_by_index(index_v) << "_" << airportgraph->get_airport_name_by_index(index_w);
    SCIP_CALL_EXC(SCIPcreateConsBasicLinear(scip, &cons2, consname_1.str().c_str(), 0, NULL, NULL, -SCIPinfinity(scip), 0));
    SCIP_CALL_EXC(SCIPaddCoefLinear(scip, cons2, z_v, -1));
    SCIP_CALL_EXC(SCIPaddCoefLinear(scip, cons2, w_vw, 1));
    SCIPaddCons(scip, cons2);
    SCIPreleaseCons(scip, &cons2);
    // w_vw < z_w
    SCIP_Cons *cons3;
    std::ostringstream consname_3;
    consname_3 << "MC_cons3_" << airportgraph->get_airport_name_by_index(index_v) << "_" << airportgraph->get_airport_name_by_index(index_w);
    SCIP_CALL_EXC(SCIPcreateConsBasicLinear(scip, &cons3, consname_1.str().c_str(), 0, NULL, NULL, -SCIPinfinity(scip), 0));
    SCIP_CALL_EXC(SCIPaddCoefLinear(scip, cons3, z_w, -1));
    SCIP_CALL_EXC(SCIPaddCoefLinear(scip, cons3, w_vw, 1));
    SCIPaddCons(scip, cons3);
    SCIPreleaseCons(scip, &cons3);
    return w_vw;
}

void create_bilinear_variables(AirportGraph *airportgraph, SCIP *scip)
{
    auto graph = airportgraph->graph;
    for (int v = 0; v < airportgraph->number_of_airports; v++)
    {
        for (auto w = v + 1; w < airportgraph->number_of_airports; w++)
        {
            auto edge = boost::edge(airportgraph->node_vector[v], airportgraph->node_vector[w], graph);
            assert(edge.second = true);
            // falls elec geflogen werden kann erstelle constraints:
            for (auto t = 0; t < airportgraph->aircrafts.size(); t++)
            {
                if (airportgraph->aircrafts[t].is_electric && 0 < graph[edge.first].costs[t])
                {
                    auto w_vw = add_one_bilinear_var_with_mccormick_constraints(airportgraph, scip, v, w);
                    auto conss = airportgraph->get_optimal_path_constraints_for_vertex_pair(v, w);
                    auto demand = airportgraph->get_demand_for_index_pair(v, w);
                    double cost = graph[edge.first].costs[t] * demand;
                    for (auto cons : conss)
                    {
                        SCIP_Bool success;
                        double rhs = SCIPconsGetRhs(scip, cons, &success);
                        assert(success);
                        SCIPaddCoefLinear(scip, cons, w_vw, rhs - cost);
                    }
                }
            }
        }
    }
}
