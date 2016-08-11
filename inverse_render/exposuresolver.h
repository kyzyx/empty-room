#ifndef _EXPOSURE_SOLVER_H
#define _EXPOSURE_SOLVER_H
#include <vector>
#include "datamanager/meshmanager.h"

enum {
    LOSS_L2,
    LOSS_CAUCHY,
    LOSS_HUBER,
};
double solveExposure(
        MeshManager* manager,
        int n,
        double* exposures,
        double* radiances,
        std::vector<int>& indices,
        int lossfn=LOSS_L2,
        float scale=5.0
        );

#endif
