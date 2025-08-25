#include "airport_graph.h"
#include "../../dump.h"
#include "../../global_parameter.h"

// Creates a graph for airport connections
AirportGraph::AirportGraph(Input_AirportGraph input)
{
    std::cout << "construction of graph: ";

    using i = index_type;

    auto path = input.path_dataset;
    elec_range = input.elec_range;
    // vertices
    this->create_vertices(path + "/" + input.dataset_name + "_dataset_airports.csv");
    // edges
    this->create_edges(path + "/" + input.dataset_name + "_dataset_aircraft.csv");
    // edge_data
    this->create_edge_data(path + "/" + input.dataset_name + "_dataset_pairwise.csv", input);

    this->create_global_data(path + "/" + input.eleccost_dataset + ".csv");

    this->compute_electrification_cost();

    this->delete_elec_edges_with_too_high_range();

    std::cout << " DONE\n";
}
// Graph Creation:
void AirportGraph::set_number_airports(int nairports)
{
    this->number_of_airports = nairports;
}
void AirportGraph::create_vertices(std::string path)
{
    int nbairports;
    std::ifstream airport(path);
    if (!airport.is_open())
    {
        std::cout << "error when opening airport data!" << endl;
        abort();
    }
    else
    {
        airport >> this->number_of_airports;
        airport.get();
        string row, name;
        for (int n = 0; n < this->number_of_airports; n++)
        {
            getline(airport, row);
            istringstream entry(row);

            getline(entry, name, ';');
            getline(entry, name, ';');
            getline(entry, name, ';');
            auto temp_node = boost::add_vertex(graph);
            this->graph[temp_node].name = name;
            this->graph[temp_node].index = n;
            this->graph[temp_node].electrifiable = false;
            this->graph[temp_node].sum_demand = 0;
            this->node_vector.push_back(temp_node);
        }
    }
}
void AirportGraph::create_edges(std::string path)
{
    std::ifstream aircraft(path);
    int number_of_aircraft;

    if (!aircraft.is_open())
    {
        std::cout << "error when opening aircraft data!" << endl;
        abort();
    }
    else
    {
        aircraft >> number_of_aircraft;
        aircraft.get();
        string row;
        for (int n = 0; n < number_of_aircraft; n++)
        {
            getline(aircraft, row);
            istringstream entry(row);
            Aircraft t;

            getline(entry, t.name, ';');
            entry >> t.range;
            entry.get();
            entry >> t.seat_capacity;
            entry.get();
            entry >> t.is_electric;
            entry.get();
            entry >> t.D1;
            entry.get();
            entry >> t.D2;
            entry.get();
            entry >> t.D3;
            entry.get();
            entry >> t.D4;
            entry.get();
            entry >> t.F1;
            entry.get();
            entry >> t.F2;
            entry.get();
            entry >> t.F3;
            entry.get();
            string temp;
            getline(entry, temp, ';');
            while (getline(entry, temp, ';'))
            {
                t.compatible_airports.push_back(atoi(temp.c_str()));
                if (t.is_electric)
                {
                    this->graph[this->node_vector[atoi(temp.c_str())]].electrifiable = true;
                }
            }
            this->aircrafts.push_back(t);
        }
    }
}
void AirportGraph::create_edge_data(std::string path, Input_AirportGraph input)
{
    std::ifstream pairwise(path);
    int number_of_connections;

    if (!pairwise.is_open())
    {
        std::cout << "error when opening pairwise data!\n";
        abort();
    }
    else
    {
        pairwise >> number_of_connections;
        pairwise.get();
        string row;
        for (int n = 0; n < number_of_connections; n++)
        {
            getline(pairwise, row);
            istringstream entry(row);
            double disttemp = 0.;
            double navitemp = 0.;
            Pairwise pair;
            int airport_1, airport_2;

            entry >> airport_1;
            entry.get();
            entry >> airport_2;
            entry.get();
            entry >> disttemp;
            entry.get();
            entry >> pair.demand;
            if (pair.demand < 0)
                continue;
            if (airport_1 < airport_2)
                path_data[index_type{airport_1, airport_2}] = pair;
            path_data[index_type{airport_1, airport_2}].demand /= constants::SCALING_FACTOR_DEMAND;
            if (airport_2 < airport_1)
                path_data[index_type{airport_2, airport_1}] = pair;
            path_data[index_type{airport_2, airport_1}].demand /= constants::SCALING_FACTOR_DEMAND;

            double p = 0.;
            this->graph[airport_1].sum_demand += pair.demand;
            this->graph[airport_2].sum_demand += pair.demand;

            vector<double> costs, emissions;
            for (int t = 0; t < this->aircrafts.size(); t++)
            {
                entry.get();
                entry >> navitemp;

                double cost = -1.;
                double emission = -1.;
                if (navitemp != -1.)
                {
                    if (this->aircrafts[t].is_electric == 0)
                        p = input.fuel_price;
                    else
                        p = input.electricity_price;
                    double p_at = this->aircrafts[t].D1 * disttemp + this->aircrafts[t].D2 * this->aircrafts[t].seat_capacity + this->aircrafts[t].D3 + this->aircrafts[t].D4 * 26;
                    cost = p * p_at + this->aircrafts[t].F1 * disttemp + this->aircrafts[t].F2 * input.seat_load_factor + this->aircrafts[t].F3 + navitemp;
                    emission = 3.15 * p_at * (1 - this->aircrafts[t].is_electric) * pow(10, -3); // scaling the amount of emitted CO2 to tonne
                }

                // Added: per_passenger:
                if (cost < 0)
                {
                    costs.push_back(cost);
                }
                else
                {
                    costs.push_back(cost / aircrafts[t].seat_capacity / constants::SCALING_FACTOR_EURO * constants::SCALING_FACTOR_DEMAND);
                }
                if (emission < 0)
                {
                    emissions.push_back(emission);
                }
                else
                {
                    emissions.push_back(emission / aircrafts[t].seat_capacity / constants::SCALING_FACTOR_EMISSION * constants::SCALING_FACTOR_DEMAND);
                }
            }

            auto temp_edge_hin = boost::add_edge(this->node_vector[int(airport_1)], this->node_vector[int(airport_2)], this->graph);
            // auto temp_edge_rueck = boost::add_edge(this->node_vector[int(airport_2)], this->node_vector[int(airport_1)], this->graph);

            this->graph[temp_edge_hin.first].index = n;
            this->graph[temp_edge_hin.first].costs = costs;
            this->graph[temp_edge_hin.first].emissions = emissions;
            this->graph[temp_edge_hin.first].distance = disttemp;
            this->graph[temp_edge_hin.first].elec_flight = (graph[temp_edge_hin.first].costs[9] <= 0) ? 0 : 1;

            // this->graph[temp_edge_rueck.first].index = n;
            // this->graph[temp_edge_rueck.first].costs = costs;
            // this->graph[temp_edge_rueck.first].emissions = emissions;
            // this->graph[temp_edge_rueck.first].distance = disttemp;
        }
    }
}
void AirportGraph::create_global_data(std::string path)
{
    int nelectriccosts;
    std::ifstream global(path);
    if (!global.is_open())
    {
        std::cout << "error when opening electrifitation data!" << endl;
        abort();
    }
    else
    {
        global >> nelectriccosts;
        string row, skip, growth, cost;
        global.get();
        for (int n = 0; n < nelectriccosts; ++n)
        {
            getline(global, row);
            istringstream entry(row);
            getline(entry, skip, ';');
            getline(entry, growth, ';');
            getline(entry, skip, ';');
            getline(entry, cost, ';');

            this->graph[graph_bundle].airport_rank.push_back(std::stod(growth));
            this->graph[graph_bundle].electrification_costs.push_back(std::stod(cost));
        }
    }
}
void AirportGraph::compute_electrification_cost()
{
    for (auto node : this->node_vector)
    {
        if (this->graph[node].electrifiable == false)
        {
            continue;
        }
        graph[node].electrification_costs = this->graph[graph_bundle].get_electrification_cost(graph[node].sum_demand) / constants::SCALING_FACTOR_EURO;
    }
}

void AirportGraph::delete_elec_edges_with_too_high_range()
{
    for (auto [ei, ei_end] = edges(this->graph); ei != ei_end; ++ei)
    {
        if (this->graph[*ei].elec_flight == 1 && this->graph[*ei].distance > this->elec_range)
        {
            this->graph[*ei].costs[9] = -1;
            this->graph[*ei].emissions[9] = -1;
            this->graph[*ei].elec_flight = 0;
        }
    }
}

// TODO: set BigM

// getter Methods:

int AirportGraph::get_index_of_vertex(Graph::vertex_descriptor v)
{
    return graph[v].index;
}
std::string AirportGraph::get_airport_name_by_index(int i)
{
    return graph[node_vector[i]].name;
}
std::string AirportGraph::get_airport_name_by_vertex(Graph::vertex_descriptor v)
{
    return graph[v].name;
}

std::vector<SCIP_Var *> AirportGraph::get_path_variables_for_index_pair(int v, int w)
{
    if (v == w || v < 0 || w < 0)
    {
        throw std::invalid_argument("received node indices");
    }
    else if (v < w)
    {
        return vertex_pair_paths_variables[index_type{v, w}];
    }
    else
    {
        return vertex_pair_paths_variables[index_type{w, v}];
    }
}

std::vector<SCIP_Cons *> AirportGraph::get_optimal_path_constraints_for_vertex_pair(int v, int w)
{
    if (v == w || v < 0 || w < 0)
    {
        throw std::invalid_argument("received node indices");
    }
    else if (v < w)
    {
        return vertex_pair_optimal_path_constraints[index_type{v, w}];
    }
    else
    {
        return vertex_pair_optimal_path_constraints[index_type{w, v}];
    }
}

int AirportGraph::get_demand_for_index_pair(int v, int w)
{
    if (v == w || v < 0 || w < 0)
    {
        throw std::invalid_argument("received incorrect node indices");
    }
    else if (v < w)
    {
        return path_data[index_type{v, w}].demand;
    }
    else
    {
        return path_data[index_type{w, v}].demand;
    }
}

SCIP_Cons *AirportGraph::get_demand_constraint_for_index_pair(int v, int w)
{
    if (v == w || v < 0 || w < 0)
    {
        throw std::invalid_argument("received incorrect node indices");
    }
    else if (v < w)
    {
        return index_pair_demand_constraint[index_type{v, w}];
    }
    else
    {
        return index_pair_demand_constraint[index_type{w, v}];
    }
}

SCIP_Var *AirportGraph::get_bilinear_variable_for_index_pair(int v, int w)
{
    if (v == w || v < 0 || w < 0)
    {
        throw std::invalid_argument("received incorrect node indices");
    }
    else if (v < w)
    {
        return index_pair_biliniar_variable[index_type{v, w}];
    }
    else
    {
        return index_pair_biliniar_variable[index_type{w, v}];
    }
}

void AirportGraph::insert_path_variable_for_index_pair(int v, int w, SCIP_Var *path)
{
    if (v == w || v < 0 || w < 0)
    {
        throw std::invalid_argument("received node indices");
    }
    else if (v < w)
    {
        vertex_pair_paths_variables[index_type{v, w}].push_back(path);
    }
    else
    {
        vertex_pair_paths_variables[index_type{w, v}].push_back(path);
    }
}

void AirportGraph::insert_optimal_path_constraints_for_vertex_pair(int v, int w, SCIP_Cons *cons)

{
    if (v == w || v < 0 || w < 0)
    {
        throw std::invalid_argument("received node indices");
    }
    else if (v < w)
    {
        vertex_pair_optimal_path_constraints[index_type{v, w}].push_back(cons);
    }
    else
    {
        vertex_pair_optimal_path_constraints[index_type{w, v}].push_back(cons);
    }
}
