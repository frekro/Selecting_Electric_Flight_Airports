#pragma once

#include <boost/graph/adjacency_list.hpp>

#include <objscip/objscip.h>
#include <scip/scip.h>

#include <iostream>
#include <vector>

struct VertexProperty
{
    int index;
    std::string name;
    double sum_demand;
    bool electrifiable;
    int electrification_costs;
};

struct EdgeProperty
{
    int index;
    bool elec_flight;
    std::vector<double> costs;
    double distance;
    std::vector<double> emissions;
    int best_cost_aircraft; // for best path Constrainthandler
    double best_cost_value; // for best path Constrainthandler
    int pricing_aircraft;
    double pricing_value;
    double pricing_aircraft_cost;
    std::string edge_print_label;
};

struct GlobalProperty
{
    std::vector<double> airport_rank;
    std::vector<double> electrification_costs;
    double alpha = 1;
    double get_electrification_cost(int demand)
    {
        int i = 0;
        while (demand > airport_rank[i + 1] && i < airport_rank.size() - 1)
        {
            i++;
        }
        return electrification_costs[i];
    };
};

using Graph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, VertexProperty, EdgeProperty, GlobalProperty>;
