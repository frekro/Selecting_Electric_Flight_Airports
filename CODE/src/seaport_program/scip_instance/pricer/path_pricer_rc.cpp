#include "path_pricer_rc.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/r_c_shortest_paths.hpp>
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

SCIP_DECL_PRICERINIT(PathPricerNormal::scip_init)
{
    // Maybe set everything to tansfered_var / cons
    return SCIP_OKAY;
}

SCIP_RETCODE PathPricerNormal::perform_pricing(bool isfarkas)
{
    auto &graph = airportgraph->graph;

    double best_value;

    for (int v = 0; v < airportgraph->number_of_airports; v++)
    {

        std::vector<double> path_pricing_value(num_vertices(graph));
        std::vector<int> predecessor(num_vertices(graph));

        for (int w = v + 1; w < airportgraph->number_of_airports; w++)
        {
            auto vertex_v = airportgraph->node_vector[v];
            auto vertex_w = airportgraph->node_vector[w];

            auto demand_vw = airportgraph->get_demand_for_index_pair(v, w);
            set_edge_values(isfarkas, demand_vw, v, w);

            /*Use dijsktra: */
            boost::dijkstra_shortest_paths(graph, airportgraph->node_vector[v],
                                           weight_map(get(&EdgeProperty::pricing_value, graph)).distance_map(boost::make_iterator_property_map(path_pricing_value.begin(), get(&VertexProperty::index, graph))).predecessor_map(boost::make_iterator_property_map(predecessor.begin(), get(&VertexProperty::index, graph))));

            auto value = compute_path_price(isfarkas, v, w, path_pricing_value[w]);

            if (1e-4 < value)
            {
                std::vector<Graph::vertex_descriptor> elec_airports{};
                std::vector<int> path_vertices_index{};

                std::vector<int> used_aircrafts{};

                Graph::vertex_descriptor vertex = airportgraph->node_vector[w];
                double cost_propassenger = 0;
                double emission_propassenger = 0;
                while (vertex != airportgraph->node_vector[v])
                {
                    path_vertices_index.push_back(airportgraph->get_index_of_vertex(vertex));
                    auto edge = boost::edge(vertex, predecessor[vertex], graph);

                    assert(edge.second == true);

                    int aircraft = graph[edge.first].pricing_aircraft;
                    used_aircrafts.push_back(aircraft);
                    cost_propassenger += graph[edge.first].costs[aircraft];
                    emission_propassenger += graph[edge.first].emissions[aircraft];
                    if (airportgraph->aircrafts[aircraft].is_electric)
                    {
                        elec_airports.push_back(vertex);
                        elec_airports.push_back(predecessor[vertex]);
                    }
                    vertex = predecessor[vertex];
                }
                path_vertices_index.push_back(airportgraph->get_index_of_vertex(vertex));
                auto cost = cost_propassenger * demand_vw;
                auto emission = emission_propassenger * demand_vw;

                std::ostringstream namebuf;
                namebuf << "t_x_";
                for (int i = path_vertices_index.size() - 1; 0 < i; i--)
                {
                    namebuf << airportgraph->get_airport_name_by_index(path_vertices_index[i]) << "_" << "a" << used_aircrafts[i - 1] << "_";
                }
                namebuf << airportgraph->get_airport_name_by_vertex(path_vertices_index[0]);
                bool exists = check_var_already_priced(namebuf.str());
                if (exists)
                {

                    SCIPABORT();
                }
                insert_priced_var(v, w, value, namebuf, cost, emission, path_vertices_index, elec_airports, used_aircrafts);
            }
        }
    }

    return SCIP_OKAY;
}

/*______________________________________________________________*/
// Struct and Classes for Shortest Path with Resource Constraint

struct spp_with_res_cons
{
    spp_with_res_cons(int p = 0, int c = 0) : pricing_value(p), cost_value(c) {}
    spp_with_res_cons &operator=(const spp_with_res_cons &other)
    {
        if (this == &other)
            return *this;
        this->~spp_with_res_cons();
        new (this) spp_with_res_cons(other);
        return *this;
    }
    double pricing_value;
    double cost_value;
};

bool operator==(const spp_with_res_cons &res_cont_1, const spp_with_res_cons &res_cont_2)
{
    return (res_cont_1.pricing_value == res_cont_2.pricing_value && res_cont_1.cost_value == res_cont_2.cost_value);
}

bool operator<(const spp_with_res_cons &res_cont_1, const spp_with_res_cons &res_cont_2)
{
    if (res_cont_1.pricing_value > res_cont_2.pricing_value)
        return false;
    if (res_cont_1.pricing_value == res_cont_2.pricing_value)
        return res_cont_1.cost_value < res_cont_2.cost_value;
    return true;
}

// ResourceExtensionFunction model
class ref_spptw
{
public:
    double max_cost;
    inline bool operator()(const Graph &g,
                           spp_with_res_cons &new_cont, const spp_with_res_cons &old_cont,
                           boost::graph_traits<Graph>::edge_descriptor ed) const
    {
        const EdgeProperty &arc_prop = get(edge_bundle, g)[ed];
        const VertexProperty &vert_prop = get(vertex_bundle, g)[target(ed, g)];
        new_cont.pricing_value = old_cont.pricing_value + arc_prop.pricing_value;
        double &i_cost_value = new_cont.cost_value;
        i_cost_value = old_cont.cost_value + arc_prop.pricing_aircraft_cost;
        return i_cost_value <= max_cost ? true : false;
    }
};

// DominanceFunction model
class dominance_spptw
{
public:
    inline bool operator()(const spp_with_res_cons &res_cont_1,
                           const spp_with_res_cons &res_cont_2) const
    {
        return res_cont_1.pricing_value <= res_cont_2.pricing_value && res_cont_1.cost_value <= res_cont_2.cost_value;
    }
};

/*______________________________________________________________*/

SCIP_DECL_PRICERINIT(PathPricerWRC::scip_init)
{
    // calculate shortest path for every node pair:
    set_max_cost();
    return SCIP_OKAY;
}

void PathPricerWRC::set_max_cost()
{
    auto &graph = airportgraph->graph;
    std::vector<double> path_cost_value(num_vertices(graph));
    std::vector<int> predecessor(num_vertices(graph));
    for (auto [ei, ei_end] = edges(graph); ei != ei_end; ++ei)
    {
        double best_value = std::numeric_limits<double>::max();
        int best_aircraft = -1;
        for (int aircraft = 0; aircraft < airportgraph->aircrafts.size(); aircraft++)
        {
            if (graph[*ei].costs[aircraft] < 0 || (airportgraph->aircrafts[aircraft].is_electric))
            {
                continue;
            }
            auto value = graph[*ei].costs[aircraft];
            if (value < best_value)
            {
                best_value = value;
                best_aircraft = aircraft;
            }
        }
        graph[*ei].best_cost_aircraft = best_aircraft;
        graph[*ei].best_cost_value = best_value;
        assert(0 < best_value);
    }

    for (int v = 0; v < airportgraph->number_of_airports - 1; v++)
    {
        std::vector<double> path_cost_value(num_vertices(graph));
        std::vector<int> predecessor(num_vertices(graph));
        boost::dijkstra_shortest_paths(graph, airportgraph->node_vector[v],
                                       weight_map(get(&EdgeProperty::best_cost_value, graph)).distance_map(boost::make_iterator_property_map(path_cost_value.begin(), get(&VertexProperty::index, graph))).predecessor_map(boost::make_iterator_property_map(predecessor.begin(), get(&VertexProperty::index, graph))));

        for (int w = v + 1; w < airportgraph->number_of_airports; w++)
        {
            auto value = path_cost_value[airportgraph->node_vector[w]];
            index_pair_max_cost[index_type{v, w}] = path_cost_value[airportgraph->node_vector[w]];
            pBranchDec << airportgraph->get_airport_name_by_index(v) << ",  " << airportgraph->get_airport_name_by_index(w) << "\t\t\t| Max_Cost: " << path_cost_value[airportgraph->node_vector[w]] * airportgraph->get_demand_for_index_pair(v, w) << std::endl;
        }
    }
}

SCIP_RETCODE PathPricerWRC::perform_pricing(bool isfarkas)
{
    auto &graph = airportgraph->graph;

    for (int v = 0; v < airportgraph->number_of_airports; v++)
    {
        for (int w = v + 1; w < airportgraph->number_of_airports; w++)
        {
            auto vertex_v = airportgraph->node_vector[v];
            auto vertex_w = airportgraph->node_vector[w];

            std::vector<std::vector<graph_traits<Graph>::edge_descriptor>> opt_solutions_spptw;
            std::vector<spp_with_res_cons> pareto_opt_rcs_spptw;
            auto demand_vw = airportgraph->get_demand_for_index_pair(v, w);
            set_edge_values(isfarkas, demand_vw, v, w);
            double max_cost;
            if (isfarkas)
            {
                max_cost = std::numeric_limits<double>::max();
            }
            else
            {
                max_cost = index_pair_max_cost[index_type{v, w}];
            }

            assert(0 <= max_cost);
            r_c_shortest_paths(graph, get(&VertexProperty::index, graph),
                               get(&EdgeProperty::index, graph), vertex_v, vertex_w, opt_solutions_spptw,
                               pareto_opt_rcs_spptw, spp_with_res_cons(0, 0), ref_spptw(max_cost),
                               dominance_spptw(),
                               std::allocator<r_c_shortest_paths_label<Graph, spp_with_res_cons>>(),
                               default_r_c_shortest_paths_visitor());

            if (opt_solutions_spptw.size() == 0)
            {
                continue;
            }

            for (int k = 0; k < pareto_opt_rcs_spptw.size(); k++)
            {
                auto value = compute_path_price(isfarkas, v, w, pareto_opt_rcs_spptw[k].pricing_value);
                if (1e-4 < value)
                {
                    std::vector<Graph::vertex_descriptor> elec_airports{};
                    std::vector<int> path_vertices_index{};

                    std::vector<int> used_aircrafts{};

                    Graph::vertex_descriptor vertex = airportgraph->node_vector[w];
                    std::ostringstream namebuf;
                    namebuf << "t_x_";
                    double cost_propassenger = 0;
                    double emission_propassenger = 0;

                    for (int j = opt_solutions_spptw[k].size() - 1; 0 <= j; j--)
                    {
                        path_vertices_index.push_back(airportgraph->get_index_of_vertex(source(opt_solutions_spptw[k][j], graph)));
                        int aircraft = graph[opt_solutions_spptw[k][j]].pricing_aircraft;
                        used_aircrafts.push_back(aircraft);
                        cost_propassenger += graph[opt_solutions_spptw[k][j]].costs[aircraft];
                        emission_propassenger += graph[opt_solutions_spptw[k][j]].emissions[aircraft];
                        namebuf << airportgraph->get_airport_name_by_vertex(source(opt_solutions_spptw[k][j], graph)) << "_" << "a" << aircraft << "_";
                        if (airportgraph->aircrafts[aircraft].is_electric)
                        {
                            elec_airports.push_back(source(opt_solutions_spptw[k][j], graph));
                            elec_airports.push_back(target(opt_solutions_spptw[k][j], graph));
                        }
                    }
                    namebuf << airportgraph->get_airport_name_by_vertex(target(opt_solutions_spptw[k][0], graph));
                    path_vertices_index.push_back(airportgraph->get_index_of_vertex(target(opt_solutions_spptw[k][0], graph)));

                    assert(used_aircrafts.size() + 1 == path_vertices_index.size());
                    auto cost = cost_propassenger * demand_vw;
                    auto emission = emission_propassenger * demand_vw;
                    bool exists = check_var_already_priced(namebuf.str());
                    if (exists)
                    {
                        SCIPABORT();
                    }
                    insert_priced_var(v, w, value, namebuf, cost, emission, path_vertices_index, elec_airports, used_aircrafts);
                }
            }
        }
    }
    return SCIP_OKAY;
}
