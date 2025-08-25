#pragma once
#include "airport_graph_prop.h"
#include "data.h"

using namespace std;
using namespace boost;

struct Input_AirportGraph
{
    std::string path_dataset;
    std::string dataset_name;
    std::string eleccost_dataset;
    std::string outputfouldername;
    double elec_range;
    double fuel_price;
    double electricity_price;
    double aircraft_elec_range;
    double seat_load_factor;
};

class AirportGraph
{
private:
    double elec_range;
    sparse_array<std::vector<SCIP_Var *>> vertex_pair_paths_variables;
    sparse_array<std::vector<SCIP_Cons *>> vertex_pair_optimal_path_constraints;

    void set_number_airports(int nairports);
    void create_vertices(std::string path);
    void create_edges(std::string path);
    void create_edge_data(std::string path, Input_AirportGraph input);
    void create_global_data(std::string path);
    void compute_electrification_cost();
    void delete_elec_edges_with_too_high_range();

public:
    Graph graph;
    std::vector<Graph::vertex_descriptor> node_vector;
    vector<Aircraft> aircrafts;
    sparse_array<Pairwise> path_data;
    int number_of_airports;

    std::unordered_map<int, SCIP_Var *> map_vertex_to_variable{};
    std::unordered_map<SCIP_Var *, int> map_variable_to_vertex{};
    std::unordered_map<int, SCIP_Cons *> vertex_to_elec_constraint{};
    sparse_array<SCIP_Cons *> index_pair_demand_constraint;
    sparse_array<SCIP_Var *> index_pair_biliniar_variable;
    sparse_array<SCIP_Var *> index_pair_direct_elec_flight;

    SCIP_Cons *budget_cons;

    // Creates a fleet planning graph
    AirportGraph(Input_AirportGraph input);

    // getter_functions:
    int get_index_of_vertex(Graph::vertex_descriptor v);
    std::string get_airport_name_by_index(int i);
    std::string get_airport_name_by_vertex(Graph::vertex_descriptor v);
    std::vector<SCIP_Var *> get_path_variables_for_index_pair(int v, int w);
    std::vector<SCIP_Cons *> get_optimal_path_constraints_for_vertex_pair(int v, int w);
    SCIP_Cons *get_demand_constraint_for_index_pair(int v, int w);
    int get_demand_for_index_pair(int v, int w);
    SCIP_Var *get_bilinear_variable_for_index_pair(int v, int w);

    // insert
    void insert_path_variable_for_index_pair(int v, int w, SCIP_Var *path);
    void insert_optimal_path_constraints_for_vertex_pair(int v, int w, SCIP_Cons *cons);

    // Deconstructor
    ~AirportGraph()
    {
    }
};