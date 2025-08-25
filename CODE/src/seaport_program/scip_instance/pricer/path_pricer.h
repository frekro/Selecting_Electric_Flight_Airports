#pragma once

#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include <objscip/objscip.h>
#include "../../graph/airport_graph.h"

class PathPricer : public scip::ObjPricer
{
protected:
    double compute_cost(double Q, double phi_v, double phi_w, int demand_vw, double cost_a, double emission_a, bool iselec);
    std::pair<int, double> get_best_aircraft_on_edge(Graph::edge_descriptor edge, double Q, double phi_v, double phi_w, bool isfarkas, int demand);
    SCIP_RETCODE set_edge_values(bool isfarkas, int demand, int v, int w);
    double compute_path_price(bool isfarkas, int v, int w, double val_shortest_path);
    bool check_var_already_priced(std::string name);
    void insert_priced_var(int v, int w, double score, std::ostringstream &varname, double cost, double emission, std::vector<int> path_vertices_index, std::vector<Graph::vertex_descriptor> elec_vertices, std::vector<int> used_aircrafts);
    void print_dual_vars(bool isfarkas);
    virtual SCIP_RETCODE perform_pricing(bool isfarkas) = 0;
    SCIP *scip;
    AirportGraph *airportgraph;

public:
    PathPricer(AirportGraph *airportgraph, SCIP *scip, std::string name);

    ~PathPricer() {}

    virtual SCIP_DECL_PRICERREDCOST(scip_redcost) override;

    virtual SCIP_DECL_PRICERFARKAS(scip_farkas) override;

    virtual SCIP_DECL_PRICEREXIT(scip_exit) override;
};