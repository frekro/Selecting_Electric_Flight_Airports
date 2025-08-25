#include "costoptimalpath_cons.h"
#include "../../graph/airport_graph.h"
#include "scip/cons_linear.h"
#include "../model/scip_exception.h"
#include "../../../dump.h"
#include "../../../global.h"
#include "../../../global_parameter.h"
#include "../model/pathvar_prob.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <iostream>
#include <ranges>
#include <vector>

struct SCIP_ConsData
{
};

struct Path_Data_for_Constraint
{
    int v;
    int w;
    string namevar;
    std::vector<int> index_path;
};

static SCIP_Cons *t_c(SCIP *scip, SCIP_Cons *cons)
{
    SCIP_CONS *tcon;
    SCIP_CALL_EXC(SCIPgetTransformedCons(scip, cons, &tcon));
    return tcon;
}

static SCIP_VAR *t_v(SCIP *scip, SCIP_VAR *var)
{
    SCIP_VAR *tvar;
    SCIP_CALL_EXC(SCIPgetTransformedVar(scip, var, &tvar));
    return tvar;
}

void ConshdlrOptPaths::fix_edge_cost_graph(SCIP_Sol *sol)
{
    auto &graph = airportgraph->graph;
    for (auto [ei, ei_end] = edges(graph); ei != ei_end; ++ei)
    {
        auto vertex_v = target(*ei, graph);
        auto vertex_w = source(*ei, graph);
        auto var_vertex_v = airportgraph->map_vertex_to_variable[vertex_v];
        auto var_vertex_w = airportgraph->map_vertex_to_variable[vertex_w];
        auto val_v = SCIPgetSolVal(scip, sol, var_vertex_v);
        auto val_w = SCIPgetSolVal(scip, sol, var_vertex_w);

        if (1e-06 <= val_v && val_v <= 1 - 1e-06)
        {
            SCIPABORT();
        }
        if (1e-06 <= val_w && val_w <= 1 - 1e-06)
        {
            SCIPABORT();
        }

        double best_value = std::numeric_limits<double>::max();
        int best_aircraft = -1;
        for (int aircraft = 0; aircraft < airportgraph->aircrafts.size(); aircraft++)
        {
            if (graph[*ei].costs[aircraft] < 0 || (airportgraph->aircrafts[aircraft].is_electric && (val_v == 0 || val_w == 0)))
            {
                continue;
            }
            auto value = graph[*ei].costs[aircraft];

            if (value < best_value)
            {
                best_value = value;
                best_aircraft = aircraft;
            }
            assert(0 <= best_aircraft);
            assert(0 < best_value);
        }
        auto r_best_value = best_value;
        graph[*ei].best_cost_aircraft = best_aircraft;
        graph[*ei].best_cost_value = r_best_value;
    }
}

double ConshdlrOptPaths::get_current_cost(int v, int w, SCIP_Sol *sol)
{
    auto paths = airportgraph->get_path_variables_for_index_pair(v, w);
    double current_cost = 0;
    for (auto path : paths)
    {
        auto path_data = SCIPvarGetData(path);
        auto cost = path_data->cost; // already multiplied by the demand!
        auto val = SCIPgetSolVal(scip, sol, path);
        current_cost += cost * val;
    }
    assert(0 <= current_cost);
    return current_cost;
}

void ConshdlrOptPaths::add_constraint_for_path(Path_Data_for_Constraint data)
{
    // get Path costs if nothing is electrified:
    auto &graph = airportgraph->graph;
    std::vector<double> path_cost_without_elec{};
    std::vector<double> path_cost_with_elec{};
    auto demand = airportgraph->get_demand_for_index_pair(data.v, data.w);

    for (auto i_v = 0; i_v < data.index_path.size() - 1; i_v++)
    {
        auto edge = boost::edge(airportgraph->node_vector[data.index_path[i_v]], airportgraph->node_vector[data.index_path[i_v + 1]], graph);
        double edge_cost_min_without_elec = std::numeric_limits<double>::max();
        double edge_cost_min_with_elec = std::numeric_limits<double>::max();

        assert(edge.second == true);
        for (int t = 0; t < airportgraph->aircrafts.size(); t++)
        {
            auto name_v = airportgraph->get_airport_name_by_index(data.v);
            auto name_w = airportgraph->get_airport_name_by_index(data.w);

            if (airportgraph->aircrafts[t].is_electric)
            {
                if (graph[edge.first].costs[t] < edge_cost_min_with_elec && 0 < graph[edge.first].costs[t])
                {
                    edge_cost_min_with_elec = graph[edge.first].costs[t];
                }
            }
            else
            {
                if (graph[edge.first].costs[t] < edge_cost_min_with_elec && 0 < graph[edge.first].costs[t])
                {
                    edge_cost_min_with_elec = graph[edge.first].costs[t];
                }
                if (graph[edge.first].costs[t] < edge_cost_min_without_elec && 0 < graph[edge.first].costs[t])
                {
                    edge_cost_min_without_elec = graph[edge.first].costs[t];
                }
            }
        }
        assert(0 < edge_cost_min_without_elec);
        assert(0 < edge_cost_min_with_elec);
        assert(edge_cost_min_with_elec <= edge_cost_min_without_elec);
        path_cost_without_elec.push_back(edge_cost_min_without_elec);
        path_cost_with_elec.push_back(edge_cost_min_with_elec);
    }

    SCIP_CONS *optimal_path_constraint;
    std::ostringstream namebuf;
    namebuf << "Cost_Optimal_Path_Constraint_for_" << airportgraph->get_airport_name_by_index(data.w) << "_" << airportgraph->get_airport_name_by_index(data.v);
    SCIP_CALL_EXC(SCIPcreateConsLinear(scip,
                                       &optimal_path_constraint,
                                       namebuf.str().c_str(),
                                       0,
                                       NULL,
                                       NULL,
                                       -SCIPinfinity(scip), // lhs
                                       0,                   // rhs
                                       TRUE,                // initial
                                       TRUE,                // separate
                                       TRUE,                // enforce
                                       TRUE,                // check
                                       TRUE,                // propagate
                                       FALSE,               // local
                                       TRUE,                // modifiable
                                       FALSE,               // dynamic
                                       FALSE,               // removable
                                       FALSE));             // sticking at node
    SCIP_CALL_EXC(SCIPaddCons(scip, optimal_path_constraint));

    double rhs = 0;

    for (auto i_v = 0; i_v < data.index_path.size() - 1; i_v++)
    {
        rhs += path_cost_without_elec[i_v] * demand;
        auto edge = boost::edge(airportgraph->node_vector[data.index_path[i_v]], airportgraph->node_vector[data.index_path[i_v + 1]], graph);

        if (0 < graph[edge.first].costs[9])
        {
            auto c_fg = (path_cost_without_elec[i_v] - path_cost_with_elec[i_v]) * demand;
            auto r_c_fg = c_fg;
            auto var = airportgraph->get_bilinear_variable_for_index_pair(data.index_path[i_v], data.index_path[i_v + 1]);
            assert(var != nullptr);
            SCIP_CALL_EXC(SCIPaddCoefLinear(scip, optimal_path_constraint, var, r_c_fg));
        }
    }
    auto r_rhs = rhs;
    SCIP_CALL_EXC(SCIPchgRhsLinear(scip, optimal_path_constraint, r_rhs));

    auto paths_vw = airportgraph->get_path_variables_for_index_pair(data.v, data.w);
    for (auto path : paths_vw)
    {
        auto path_data = SCIPvarGetData(path);
        SCIP_CALL_EXC(SCIPaddCoefLinear(scip, optimal_path_constraint, path, path_data->cost));
    }

    airportgraph->insert_optimal_path_constraints_for_vertex_pair(data.v, data.w, optimal_path_constraint);
}

Path_Data_for_Constraint ConshdlrOptPaths::prepare_constraints(std::vector<int> predecessor, int v, int w)
{
    auto &graph = airportgraph->graph;
    std::vector<int> path = {};
    auto vertex = airportgraph->node_vector[w];
    while (vertex != airportgraph->node_vector[v])
    {
        auto edge = boost::edge(vertex, predecessor[vertex], graph);

        assert(edge.second == true);
        path.push_back(airportgraph->get_index_of_vertex(vertex));
        vertex = predecessor[vertex];
    }
    path.push_back(vertex);
    std::reverse(path.begin(), path.end());

    Path_Data_for_Constraint better_path{.v = v, .w = w, .index_path = path};

    return better_path;
}

std::tuple<bool, std::vector<Path_Data_for_Constraint>> ConshdlrOptPaths::check_feasibilty(SCIP_Sol *sol, bool prep_cons)
{
    fix_edge_cost_graph(sol);
    auto &graph = airportgraph->graph;
    std::vector<Path_Data_for_Constraint> better_paths;
    bool feasible = true;
    for (int v = 0; v < airportgraph->number_of_airports - 1; v++)
    {
        std::vector<double> path_cost_value(num_vertices(graph));
        std::vector<int> predecessor(num_vertices(graph));
        boost::dijkstra_shortest_paths(graph, airportgraph->node_vector[v],
                                       weight_map(get(&EdgeProperty::best_cost_value, graph)).distance_map(boost::make_iterator_property_map(path_cost_value.begin(), get(&VertexProperty::index, graph))).predecessor_map(boost::make_iterator_property_map(predecessor.begin(), get(&VertexProperty::index, graph))));
        for (int w = v + 1; w < airportgraph->number_of_airports; w++)
        {
            auto demand = airportgraph->get_demand_for_index_pair(v, w);
            auto cost_vw = demand * path_cost_value[w];
            auto current_cost_vw = get_current_cost(v, w, sol);
            if (cost_vw < current_cost_vw - 1e-4)
            {

                feasible = false;
                if (prep_cons)
                {
                    better_paths.push_back(prepare_constraints(predecessor, v, w));
                }
                else
                {
                    return make_tuple(false, better_paths);
                }
            }
        }
    }
    return std::make_tuple(feasible, better_paths);
}
/*
The CONSCHECK callback gets a primal solution candidate in a SCIP_SOL* data structure
and has to check this solution for global feasibility. It has to return a result SCIP_FEASIBLE,
if the solution satisfies all the constraints of the constraint handler, and a result SCIP_INFEASIBLE
if there is at least one constraint that is violated.
*/
SCIP_DECL_CONSCHECK(ConshdlrOptPaths::scip_check)
{
    assert(result != NULL);
    *result = SCIP_FEASIBLE;

    auto feas = check_feasibilty(sol, true);
    auto feasible = std::get<0>(feas);
    if (!feasible)
    {
        *result = SCIP_INFEASIBLE;
        // return SCIP_OKAY;
    }
    return SCIP_OKAY;
}

/*The CONSENFOLP method is called after the price-and-cut loop was finished and an LP solution is available.*/

SCIP_DECL_CONSENFOLP(ConshdlrOptPaths::scip_enfolp)
{ /*lint --e{715}*/
    assert(result != NULL);
    auto feas = check_feasibilty(NULL, true);
    auto feasible = std::get<0>(check_feasibilty(NULL, false));
    if (feasible == true)
    {
        *result = SCIP_FEASIBLE;
        return SCIP_OKAY;
    }
    for (auto path : std::get<1>(feas))
    {
        add_constraint_for_path(path);
    }

    *result = SCIP_CONSADDED;

    return SCIP_OKAY;
}

SCIP_DECL_CONSENFOPS(ConshdlrOptPaths::scip_enfops)
{ /*lint --e{715}*/
    assert(result != NULL);

    *result = SCIP_INFEASIBLE;
    auto feas = check_feasibilty(NULL, true);
    auto feasible = std::get<0>(check_feasibilty(NULL, false));
    if (feasible == true)
    {
        *result = SCIP_FEASIBLE;
        return SCIP_OKAY;
    }
    for (auto path : std::get<1>(feas))
    {
        add_constraint_for_path(path);
    }

    return SCIP_OKAY;
}

SCIP_DECL_CONSLOCK(ConshdlrOptPaths::scip_lock)
{ /*lint --e{715}*/

    auto &graph = airportgraph->graph;

    for (int v = 0; v < airportgraph->number_of_airports; v++)
    {
        if (!graph[airportgraph->node_vector[v]].electrifiable)
        {
            continue;
        }
        auto var = airportgraph->map_vertex_to_variable[airportgraph->node_vector[v]];
        SCIP_CALL(SCIPaddVarLocksType(scip, var, SCIP_LOCKTYPE_MODEL, nlockspos, nlocksneg));

        for (int w = v + 1; w < airportgraph->number_of_airports; w++)
        {
            {
                auto path = airportgraph->index_pair_direct_elec_flight[index_type{v, w}];
                if (path != NULL)
                {
                    SCIP_CALL(SCIPaddVarLocksType(scip, path, SCIP_LOCKTYPE_MODEL, nlocksneg, nlockspos));
                }
            }
        }
    }
    return SCIP_OKAY;
}

SCIP_DECL_CONSTRANS(ConshdlrOptPaths::scip_trans)
{
    SCIP_CONSDATA *sourcedata;
    SCIP_CONSDATA *targetdata = NULL;

    sourcedata = SCIPconsGetData(sourcecons);
    assert(sourcedata != NULL);

    SCIP_CALL(SCIPallocBlockMemory(scip, &targetdata));

    /* create target constraint */
    SCIP_CALL(SCIPcreateCons(scip, targetcons, SCIPconsGetName(sourcecons), conshdlr, targetdata,
                             SCIPconsIsInitial(sourcecons), SCIPconsIsSeparated(sourcecons), SCIPconsIsEnforced(sourcecons),
                             SCIPconsIsChecked(sourcecons), SCIPconsIsPropagated(sourcecons), SCIPconsIsLocal(sourcecons),
                             SCIPconsIsModifiable(sourcecons), SCIPconsIsDynamic(sourcecons), SCIPconsIsRemovable(sourcecons),
                             SCIPconsIsStickingAtNode(sourcecons)));

    return SCIP_OKAY;
};

SCIP_DECL_CONSPRINT(ConshdlrOptPaths::scip_print)
{ /*lint --e{715}*/
    SCIP_CONSDATA *consdata;

    SCIPinfoMessage(scip, file, "OPTPath_Constraint: ");

    return SCIP_OKAY;
}

SCIP_RETCODE SCIPcreateConsCostOptPaths(
    SCIP *scip,           /**< SCIP data structure */
    SCIP_CONS **cons,     /**< pointer to hold the created constraint */
    const char *name,     /**< name of constraint */
    SCIP_Bool initial,    /**< should the LP relaxation of constraint be in the initial LP? */
    SCIP_Bool separate,   /**< should the constraint be separated during LP processing? */
    SCIP_Bool enforce,    /**< should the constraint be enforced during node processing? */
    SCIP_Bool check,      /**< should the constraint be checked for feasibility? */
    SCIP_Bool propagate,  /**< should the constraint be propagated during node processing? */
    SCIP_Bool local,      /**< is constraint only valid locally? */
    SCIP_Bool modifiable, /**< is constraint modifiable (subject to column generation)? */
    SCIP_Bool dynamic,    /**< is constraint dynamic? */
    SCIP_Bool removable   /**< should the constraint be removed from the LP due to aging or cleanup? */
)
{
    SCIP_CONSHDLR *conshdlr;
    SCIP_CONSDATA *consdata;

    /* find the subtour constraint handler */
    conshdlr = SCIPfindConshdlr(scip, "optpaths");
    if (conshdlr == NULL)
    {
        SCIPerrorMessage("optpaths constraint handler not found\n");
        return SCIP_PLUGINNOTFOUND;
    }

    /* create constraint data */

    /* create constraint */
    SCIP_CALL(SCIPcreateCons(scip, cons, name, conshdlr, consdata, initial, separate, enforce, check, propagate,
                             local, modifiable, dynamic, removable, FALSE));

    return SCIP_OKAY;
}