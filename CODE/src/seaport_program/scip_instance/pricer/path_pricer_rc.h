#pragma once

#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include <objscip/objscip.h>
#include "../../graph/airport_graph.h"
#include "../../graph/data.h"
#include "path_pricer.h"

class PathPricerNormal : public PathPricer
{
protected:
    SCIP_RETCODE perform_pricing(bool isfarkas) override;

public:
    using PathPricer::PathPricer;

    virtual SCIP_DECL_PRICERINIT(scip_init) override;
    virtual ~PathPricerNormal() = default;
};

class PathPricerWRC : public PathPricer
{
protected:
    sparse_array<double> index_pair_max_cost;

    void set_max_cost();

    SCIP_RETCODE perform_pricing(bool isfarkas) override;

public:
    using PathPricer::PathPricer;
    virtual SCIP_DECL_PRICERINIT(scip_init) override;
    virtual ~PathPricerWRC() = default;
};