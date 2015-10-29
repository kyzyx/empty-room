#ifndef _EXPOSURE_SOLVER_H
#define _EXPOSURE_SOLVER_H
#include <vector>
#include "datamanager/meshmanager.h"

double solveExposure(
        MeshManager* manager,
        int n,
        double* exposures,
        double* radiances,
        std::vector<int>& indices
        );

#endif
