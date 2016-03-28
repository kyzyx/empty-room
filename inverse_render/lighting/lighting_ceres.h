#ifndef _LIGHT_CERES_H
#define _LIGHT_CERES_H
#include "light.h"

namespace ceres {
    class Problem;
}

void addCeres(Light* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);
void addCeresArea(AreaLight* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);
void addCeresSH(SHLight* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);
void addCeresCubemap(CubemapLight* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);

#endif
