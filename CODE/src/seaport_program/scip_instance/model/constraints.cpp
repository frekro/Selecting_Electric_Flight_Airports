#include "constraints.h"
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include "../../seaport_class.h"
#include "../../graph/airport_graph.h"
#include "../constraint_handlers/costoptimalpath_cons.h"

#include "scip_exception.h"

// Demand Constraints (for each node Pair)

void create_path_demand_constraint_for_pair(AirportGraph *airportgraph, SCIP *scip, int v, int w, string name_v, string name_w)
{
    if (v == w)
    {
        throw std::invalid_argument("received node indices");
    }

    SCIP_CONS *path_con;
    std::ostringstream namebuf;
    namebuf << "path_demand_" << name_v << "_" << name_w;
    SCIP_CALL_EXC(SCIPcreateConsLinear(scip,
                                       &path_con,
                                       namebuf.str().c_str(),
                                       0,
                                       NULL,
                                       NULL,
                                       1,       // lhs
                                       1,       // rhs
                                       TRUE,    // initial
                                       TRUE,    // separate
                                       TRUE,    // enforce
                                       TRUE,    // check
                                       TRUE,    // propagate
                                       FALSE,   // local
                                       TRUE,    // modifiable
                                       FALSE,   // dynamic
                                       FALSE,   // removable
                                       FALSE)); // sticking at node

    SCIP_CALL_EXC(SCIPaddCons(scip, path_con));
    if (v < w)
    {
        airportgraph->index_pair_demand_constraint[index_type{v, w}] = path_con;
    }
    else
    {
        airportgraph->index_pair_demand_constraint[index_type{w, v}] = path_con;
    }
}

void create_demand_constraints(AirportGraph *airportgraph, SCIP *scip)
{

    for (int v = 0; v < airportgraph->number_of_airports; v++)
    {
        for (int w = v + 1; w < airportgraph->number_of_airports; w++)
        {
            create_path_demand_constraint_for_pair(airportgraph, scip, v, w, airportgraph->get_airport_name_by_index(v), airportgraph->get_airport_name_by_index(w));
        }
    }
}

// Electrification Constraint (for each Node)
void create_elec_cons_for_vertex(AirportGraph *airportgraph, SCIP *scip, Graph::vertex_descriptor vertex, int i, std::string vertex_name)
{
    SCIP_CONS *electrifying_con;
    std::ostringstream namebuf;
    namebuf << "electrify_" << vertex_name;
    SCIP_CALL_EXC(SCIPcreateConsLinear(scip,
                                       &electrifying_con,
                                       namebuf.str().c_str(),
                                       0,
                                       NULL,
                                       NULL,
                                       -SCIPinfinity(scip), // lhs
                                       0.,                  // rhs
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

    SCIPaddCons(scip, electrifying_con);
    airportgraph->vertex_to_elec_constraint[airportgraph->get_index_of_vertex(vertex)] = electrifying_con;
}

void create_elec_cons(AirportGraph *airportgraph, SCIP *scip)
{
    for (int v = 0; v < airportgraph->number_of_airports; v++)
    {
        auto vertex = airportgraph->node_vector[v];
        if (airportgraph->graph[vertex].electrifiable)
        {
            create_elec_cons_for_vertex(airportgraph, scip, vertex, v, airportgraph->graph[vertex].name);
        }
    }
}

// Emission Constraint
void create_budget_constraint(AirportGraph *airportgraph, SCIP *scip, double budget)
{
    SCIP_CONS *emission_con;
    std::ostringstream namebuf;
    namebuf << "budget";

    SCIP_CALL_EXC(SCIPcreateConsLinear(scip,
                                       &emission_con,
                                       namebuf.str().c_str(),
                                       0,
                                       NULL,
                                       NULL,
                                       -SCIPinfinity(scip), // lhs
                                       budget,              // rhs
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
    SCIP_CALL_EXC(SCIPaddCons(scip, emission_con));
    airportgraph->budget_cons = emission_con;
}

SCIP_CONS *create_opt_path_cons_for_index_pair(AirportGraph *airportgraph, SCIP *scip, int v, int w)
{

    SCIP_CONS *optimal_path_constraint;
    std::ostringstream namebuf;
    namebuf << "Cost_Optimal_Path_Constraint_for_" << airportgraph->get_airport_name_by_index(v) << "_" << airportgraph->get_airport_name_by_index(w);
    auto &graph = airportgraph->graph;
    auto edge = boost::edge(airportgraph->node_vector[v], airportgraph->node_vector[w], graph);

    double cost_min_without_elec = std::numeric_limits<double>::max();
    for (int t = 0; t < airportgraph->aircrafts.size(); t++)
    {
        if (!airportgraph->aircrafts[t].is_electric)
        {
            if (graph[edge.first].costs[t] < cost_min_without_elec && 0 < graph[edge.first].costs[t])
            {
                cost_min_without_elec = graph[edge.first].costs[t];
            }
        }
    }
    assert(0 < cost_min_without_elec);
    assert(cost_min_without_elec < std::numeric_limits<double>::max());

    auto demand = airportgraph->get_demand_for_index_pair(v, w);

    SCIP_CALL_EXC(SCIPcreateConsLinear(scip,
                                       &optimal_path_constraint,
                                       namebuf.str().c_str(),
                                       0,
                                       NULL,
                                       NULL,
                                       -SCIPinfinity(scip),            // lhs
                                       cost_min_without_elec * demand, // rhs
                                       TRUE,                           // initial
                                       TRUE,                           // separate
                                       TRUE,                           // enforce
                                       TRUE,                           // check
                                       TRUE,                           // propagate
                                       FALSE,                          // local
                                       TRUE,                           // modifiable
                                       FALSE,                          // dynamic
                                       FALSE,                          // removable
                                       FALSE));                        // sticking at node
    return optimal_path_constraint;
}

// Elec_Bilinear_Constraint

void create_cost_optimal_path_constraints(AirportGraph *airportgraph, SCIP *scip)
{
    SCIP_CONS *cons;

    SCIP_CALL_EXC(SCIPcreateConsCostOptPaths(scip, &cons, "cost_optimal_path", TRUE, TRUE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE));
    SCIP_CALL_EXC(SCIPaddCons(scip, cons));
    SCIP_CALL_EXC(SCIPreleaseCons(scip, &cons));
}
