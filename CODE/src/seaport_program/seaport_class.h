#pragma once
#include "graph/airport_graph.h"
#include <scip/scip.h>

class PathPricer;
class ConshdlrOptPaths;

struct Input_SeaportProgram
{
    AirportGraph *airportgraph;
    double budget;
};

class SeaportProgram
{
private:
    std::unique_ptr<PathPricer> pathpricer;
    std::unique_ptr<ConshdlrOptPaths> optpath_conshdlr;

    sparse_array<double> index_pair_max_cost;
    sparse_array<double> index_pair_max_cost_non_elec;
    std::vector<int> bigMs;

public:
    AirportGraph *airportgraph;
    SCIP *model;

    long long budget;
    int bigM;

    SeaportProgram(Input_SeaportProgram input);
    ~SeaportProgram();
};
