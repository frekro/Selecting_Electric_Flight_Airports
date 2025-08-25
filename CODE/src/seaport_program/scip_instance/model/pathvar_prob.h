#pragma once
#include <vector>
#include "scip/scip.h"

struct SCIP_VarData
{
    SCIP *scip;
    std::vector<int> vertices;
    std::vector<int> aircrafts;
    double cost;
    double emission;
};

SCIP_VAR *createPathVar(SCIP *scip, const std::string &name, double lb, double ub, double obj, SCIP_VARTYPE vartype, const std::vector<int> vertices, std::vector<int> aircrafts, double cost, double emission);