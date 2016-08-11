#ifndef _LIGHT_CERES_H
#define _LIGHT_CERES_H
#include "light.h"

namespace ceres {
    class Problem;
}

void addObservedConstraints(const Light* light,
        ceres::Problem* problem,
        double* lightarr,
        int n,
        double reg,
        std::vector<Eigen::Vector3f>& directions,
        std::vector<Eigen::Vector3f>& colors,
        std::vector<double>& weights,
        int ch=-1,
        int idx=0);
void addObservedConstraintsCubemap(const CubemapLight* light,
        ceres::Problem* problem,
        double* lightarr,
        int n,
        double reg,
        std::vector<Eigen::Vector3f>& directions,
        std::vector<Eigen::Vector3f>& colors,
        std::vector<double>& weights,
        int ch=-1,
        int idx=0);
void addObservedConstraintsArea(const AreaLight* light,
        ceres::Problem* problem,
        double* lightarr,
        int n,
        double reg,
        std::vector<Eigen::Vector3f>& directions,
        std::vector<Eigen::Vector3f>& colors,
        std::vector<double>& weights,
        int ch=-1,
        int idx=0);
void addCeres(const Light* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);
void addCeresSymmetricPoint(const SymmetricPointLight* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);
void addCeresLine(const LineLight* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);
void addCeresArea(const AreaLight* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);
void addCeresSH(const SHLight* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);
void addCeresCubemap(const CubemapLight* light, ceres::Problem* problem, double* lightarr, int n, int idx=0);

#endif
