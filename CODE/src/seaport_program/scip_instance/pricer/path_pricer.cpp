#include "path_pricer.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "../model/scip_exception.h"
#include "../model/pathvar_prob.h"
#include "../../../dump.h"
#include "../../../global_parameter.h"
#include "../../../global.h"

#include <scip/scip.h>

static SCIP_Cons *t_c(SCIP *scip, SCIP_Cons *cons)
{
    SCIP_CONS *tcon;
    SCIP_CALL_EXC(SCIPgetTransformedCons(scip, cons, &tcon));
    return tcon;
}

static SCIP_VAR *scip_create_var_continous(SCIP *scip, std::ostringstream &namebuf, double lb, double ub, double obj, bool removable)
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

PathPricer::PathPricer(AirportGraph *airportgraph, SCIP *scip, std::string name) : scip::ObjPricer(scip,
                                                                                                   name.c_str(),
                                                                                                   "Finds paths with lower operating costs.",
                                                                                                   0,
                                                                                                   TRUE),
                                                                                   airportgraph(airportgraph),
                                                                                   scip(scip)
{
}
SCIP_DECL_PRICERREDCOST(PathPricer::scip_redcost)
{
    *result = SCIP_SUCCESS;
    SCIP_CALL(perform_pricing(false));
    return SCIP_OKAY;
}

inline SCIP_DECL_PRICERFARKAS(PathPricer::scip_farkas)
{
    *result = SCIP_SUCCESS;
    SCIP_CALL(perform_pricing(true));
    return SCIP_OKAY;
}

SCIP_DECL_PRICEREXIT(PathPricer::scip_exit)
{
    return SCIP_OKAY;
}

double PathPricer::compute_cost(double Q, double phi_v, double phi_w, int demand_vw, double cost_a, double emission_a, bool iselec)
{
    assert(Q <= 0);
    assert(phi_v <= 0);
    assert(phi_w <= 0);
    return ((-Q) * cost_a + emission_a) * demand_vw - iselec * (phi_v + phi_w);
}

std::pair<int, double> PathPricer::get_best_aircraft_on_edge(Graph::edge_descriptor edge, double Q, double phi_v, double phi_w, bool isfarkas, int demand)
{
    assert(Q <= 0);
    assert(phi_v <= 0);
    assert(phi_w <= 0);

    auto &graph = airportgraph->graph;

    double best_value = std::numeric_limits<double>::max();
    int best_aircraft = -1;
    for (int aircraft = 0; aircraft < airportgraph->aircrafts.size(); aircraft++)
    {
        bool iselec = airportgraph->aircrafts[aircraft].is_electric;
        double cost = graph[edge].costs[aircraft];
        double emission = graph[edge].emissions[aircraft];
        if (0 <= cost && 0 <= emission)
        {
            emission = (1 - isfarkas) * emission;
            double value = compute_cost(Q, phi_v, phi_w, demand, cost, emission, iselec);
            if (value < best_value)
            {
                best_aircraft = aircraft;
                best_value = value;
            }
        }
    }
    assert(best_aircraft != -1);
    assert(0 <= best_value);
    return std::make_pair(best_aircraft, best_value);
}

SCIP_RETCODE PathPricer::set_edge_values(bool isfarkas, int demand, int v, int w)
{
    auto &graph = airportgraph->graph;

    for (auto [ei, ei_end] = edges(graph); ei != ei_end; ++ei)
    {
        auto vertex_v = target(*ei, graph);
        auto vertex_w = source(*ei, graph);
        auto index_v = airportgraph->get_index_of_vertex(vertex_v);
        auto index_w = airportgraph->get_index_of_vertex(vertex_w);

        auto elec_cons_v = airportgraph->vertex_to_elec_constraint[index_v];
        auto elec_cons_w = airportgraph->vertex_to_elec_constraint[index_w];

        double Q, phi_v, phi_w;

        auto opt_path_conss = airportgraph->get_optimal_path_constraints_for_vertex_pair(v, w);
        Q = 0;

        for (auto cons : opt_path_conss)
        {
            if (isfarkas)
            {
                auto val = SCIPgetDualfarkasLinear(scip, t_c(scip, cons));

                assert(val < 1e-06);
                Q += val;
            }
            else
            {
                auto val = SCIPgetDualsolLinear(scip, t_c(scip, cons));
                assert(val < 1e-06);
                Q += val;
            }
        }

        if (0 < Q && Q < 1e-06)
        {
            Q = 0;
        }

        if (isfarkas)
        {
            phi_v = SCIPgetDualfarkasLinear(scip, t_c(scip, elec_cons_v)); // - phi_v
            phi_w = SCIPgetDualfarkasLinear(scip, t_c(scip, elec_cons_w)); // - phi_w
        }
        else
        {
            phi_v = SCIPgetDualsolLinear(scip, t_c(scip, elec_cons_v)); // -phi_v
            phi_w = SCIPgetDualsolLinear(scip, t_c(scip, elec_cons_w)); // -phi_w
        }

        if (-1e-06 <= phi_v && phi_v <= 1e-06)
        {
            phi_v = 0;
        }
        if (-1e-06 <= phi_w && phi_w <= 1e-06)
        {

            phi_w = 0;
        }

        auto x = airportgraph->get_index_of_vertex(vertex_v);
        auto y = airportgraph->get_index_of_vertex(vertex_w);
        assert(0 < demand);

        auto best_aircraft = get_best_aircraft_on_edge(*ei, Q, phi_v, phi_w, isfarkas, demand);
        graph[*ei].pricing_aircraft = best_aircraft.first;
        graph[*ei].pricing_value = best_aircraft.second;
        graph[*ei].pricing_aircraft_cost = graph[*ei].costs[best_aircraft.first];

        assert(0 <= graph[*ei].pricing_value);
    }

    return SCIP_OKAY;
}

double PathPricer::compute_path_price(bool isfarkas, int v, int w, double val_shortest_path)
{
    auto &graph = airportgraph->graph;
    auto demand_cons = airportgraph->get_demand_constraint_for_index_pair(v, w);
    double pi_vw;

    assert(airportgraph->node_vector[v] != airportgraph->node_vector[w]);
    assert(airportgraph->get_demand_constraint_for_index_pair(v, w) != NULL);

    if (isfarkas)
    {
        pi_vw = SCIPgetDualfarkasLinear(scip, t_c(scip, demand_cons));
    }
    else
    {
        pi_vw = SCIPgetDualsolLinear(scip, t_c(scip, demand_cons));
    }

    if (pi_vw < 1e-06)
        pi_vw = 0;

    auto value = pi_vw - val_shortest_path;

    return value;
}

void PathPricer::insert_priced_var(int v, int w, double score, std::ostringstream &varname, double cost, double emission, std::vector<int> path_vertices_index, std::vector<Graph::vertex_descriptor> elec_vertices, std::vector<int> used_aircrafts)
{
    auto r_cost = cost;
    auto r_emission = emission;

    SCIP_Var *path;
    path = createPathVar(scip, varname.str(), 0, SCIPinfinity(scip), emission, SCIP_VARTYPE_CONTINUOUS, path_vertices_index, used_aircrafts, r_cost, emission);
    assert(path != nullptr);
    SCIP_CALL_EXC(SCIPaddPricedVar(scip, path, score));

    auto demand_cons = airportgraph->get_demand_constraint_for_index_pair(v, w);
    SCIP_CALL_EXC(SCIPaddCoefLinear(scip, t_c(scip, demand_cons), path, 1));

    if (elec_vertices.size() > 0)
    {
        for (auto vertex : elec_vertices)
        {
            auto elec_cons = airportgraph->vertex_to_elec_constraint[vertex];
            SCIP_CALL_EXC(SCIPaddCoefLinear(scip, t_c(scip, elec_cons), path, 1));
        }
    }

    auto opt_path_conss = airportgraph->get_optimal_path_constraints_for_vertex_pair(v, w);
    for (auto cons : opt_path_conss)
    {
        SCIP_CALL_EXC(SCIPaddCoefLinear(scip, t_c(scip, cons), path, cost));
    }

    pFilepriced << varname.str() << "\t\t\t\t\t\t\t\t\t\t\t|\tPricing_Value: " << score << "\t\t\t\t\t\t\t\t\t\t\t|\tCost: " << cost << "\n";
    pFilepriced.flush();
    auto optimal_paths_conss = airportgraph->get_optimal_path_constraints_for_vertex_pair(v, w);
    if (optimal_paths_conss.size() > 0)
    {
        for (auto cons : optimal_paths_conss)
        {
            SCIP_CALL_EXC(SCIPlockVarCons(scip, path, cons, FALSE, TRUE));
        }
    }
    airportgraph->insert_path_variable_for_index_pair(v, w, path);
    SCIP_CALL_EXC(SCIPreleaseVar(scip, &path));
}

bool PathPricer::check_var_already_priced(std::string name)
{
    return SCIPfindVar(scip, name.c_str());
}

void PathPricer::print_dual_vars(bool isfarkas)
{
    auto graph = airportgraph->graph;
    // node:
    for (int v = 0; v < airportgraph->number_of_airports; v++)
    {
        for (int w = v + 1; w < airportgraph->number_of_airports; w++)
        {
            auto elec_cons_v = airportgraph->vertex_to_elec_constraint[airportgraph->get_index_of_vertex(v)];
            auto elec_cons_w = airportgraph->vertex_to_elec_constraint[airportgraph->get_index_of_vertex(w)];
            auto edge = boost::edge(airportgraph->node_vector[v], airportgraph->node_vector[w], graph);
            double pi_vw, phi_v, phi_w, rho, cost, emission, demand;
            auto demand_cons = airportgraph->get_demand_constraint_for_index_pair(v, w);
            auto demand_vw = airportgraph->get_demand_for_index_pair(v, w);
            auto value = graph[edge.first].pricing_value;
            auto cost_best = graph[edge.first].costs[graph[edge.first].pricing_aircraft];
            auto emission_best = graph[edge.first].emissions[graph[edge.first].pricing_aircraft];

            if (isfarkas)
            {
                pi_vw = SCIPgetDualfarkasLinear(scip, t_c(scip, demand_cons));
                rho = SCIPgetDualfarkasLinear(scip, t_c(scip, airportgraph->budget_cons));
                phi_v = SCIPgetDualfarkasLinear(scip, t_c(scip, elec_cons_v));
                phi_w = SCIPgetDualfarkasLinear(scip, t_c(scip, elec_cons_w));
            }
            else
            {
                pi_vw = SCIPgetDualsolLinear(scip, t_c(scip, demand_cons));
                rho = SCIPgetDualsolLinear(scip, t_c(scip, airportgraph->budget_cons));
                phi_v = SCIPgetDualsolLinear(scip, t_c(scip, elec_cons_v));
                phi_w = SCIPgetDualsolLinear(scip, t_c(scip, elec_cons_w));
            }
        }
    }
}