#include "pathvar_prob.h"
#include <string>
#include "scip_exception.h"
#include "../../../dump.h"

SCIP_VAR *createPathVar(SCIP *scip, const std::string &name, double lb, double ub, double obj, SCIP_VARTYPE vartype, const std::vector<int> vertices, const std::vector<int> aircrafts, double cost, double emission)
{
    assert(0 < cost);
    SCIP_VarData *custom_data = new SCIP_VarData();
    custom_data->vertices = vertices;
    custom_data->cost = cost;
    custom_data->emission = emission;
    custom_data->aircrafts = aircrafts;

    SCIP_VAR *path;
    SCIP_CALL_EXC(SCIPcreateVar(scip,
                                &path,
                                name.c_str(),
                                lb,      // lower bound
                                ub,      // upper bound
                                obj,     // objective
                                vartype, // variable type
                                true, false, NULL, NULL, NULL, NULL, custom_data));

    return path;
}