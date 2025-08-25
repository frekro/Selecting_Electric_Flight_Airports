#pragma once

#include "objscip/objscip.h"
#include <vector>

class AirportGraph;
struct Path_Data_for_Constraint;

class ConshdlrOptPaths : public scip::ObjConshdlr
{
private:
    SCIP *scip;
    AirportGraph *airportgraph;
    std::vector<SCIP_CONS *> subset_cons;
    std::tuple<bool, std::vector<Path_Data_for_Constraint>> check_feasibilty(SCIP_Sol *sol, bool prep_cons);
    void fix_edge_cost_graph(SCIP_Sol *sol);
    double get_current_cost(int v, int w, SCIP_Sol *sol);
    Path_Data_for_Constraint prepare_constraints(std::vector<int> predecessor, int v, int w);
    void add_constraint_for_path(Path_Data_for_Constraint data);

public:
    ConshdlrOptPaths(SCIP *scip, AirportGraph *airportgraph) : ObjConshdlr(scip, "optpaths", "Forcing Cost Optimal Path Constraints",
                                                                           -1000, -1000001, -1000001, 1, -1, 1, 0,
                                                                           FALSE, FALSE, FALSE, SCIP_PROPTIMING_AFTERLPNODE, SCIP_PRESOLTIMING_FAST),
                                                               scip(scip),
                                                               airportgraph(airportgraph)
    {
    }
    /** destructor */
    virtual ~ConshdlrOptPaths()
    {
    }

    /** frees specific constraint data */

    virtual SCIP_DECL_CONSENFOLP(scip_enfolp);

    virtual SCIP_DECL_CONSENFOPS(scip_enfops);

    virtual SCIP_DECL_CONSCHECK(scip_check);

    virtual SCIP_DECL_CONSLOCK(scip_lock);

    virtual SCIP_DECL_CONSTRANS(scip_trans);

    virtual SCIP_DECL_CONSPRINT(scip_print);
};

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
);