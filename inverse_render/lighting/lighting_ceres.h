#ifndef _LIGHT_CERES_H
#define _LIGHT_CERES_H
#include "light.h"

namespace ceres {
    class Problem;
}

void addCeres(const Light* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);
void addCeresLine(const LineLight* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);
void addCeresArea(const AreaLight* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);
void addCeresSH(const SHLight* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);
void addCeresCubemap(const CubemapLight* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);

#endif
