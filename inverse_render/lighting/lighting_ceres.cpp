#include "lighting_ceres.h"
#include "ceres/ceres.h"
#include "sh.h"
#include "cubemap.h"

#include "ceres_reg.h"

using namespace std;

void addCeres(Light* light, ceres::Problem* problem, double* lightarr, int n, int idx)
{
    switch (light->typeId()) {
        case LIGHTTYPE_SH:
        case LIGHTTYPE_SH | LIGHTTYPE_POINT:
            addCeresSH((SHLight*) light, problem, lightarr, n, idx);
            break;
        case LIGHTTYPE_ENVMAP:
        case LIGHTTYPE_ENVMAP | LIGHTTYPE_POINT:
            addCeresCubemap((CubemapLight*) light, problem, lightarr, n, idx);
            break;
        case LIGHTTYPE_ENVMAP | LIGHTTYPE_LINE:
        //case LIGHTTYPE_ENVMAP | LIGHTTYPE_LINE:
        case LIGHTTYPE_LINE:
            addCeresLine((LineLight*) light, problem, lightarr, n, idx);
            break;
        case LIGHTTYPE_AREA:
            addCeresArea((AreaLight*) light, problem, lightarr, n, idx);
            break;
    }
}

void addCeresArea(AreaLight* light, ceres::Problem* problem, double* lightarr, int n, int idx)
{
    problem->SetParameterLowerBound(lightarr, idx, 0);
}

void addCeresSH(SHLight* light, ceres::Problem* problem, double* lightarr, int n, int idx)
{
    double r = light->getRegularization();
    if (r > 0) {
        for (int i = 1; i < light->numParameters(); i++) {
            problem->AddResidualBlock(CreateSHRegularizer(n, i+idx, r), NULL, lightarr);
        }
    }
}

void addCeresLine(LineLight* light, ceres::Problem* problem, double* lightarr, int n, int idx)
{
    double r = light->getRegularization();
    if (r > 0) {
        for (int i = 0; i < light->numParameters(); i++) {
            problem->AddResidualBlock(CreateSmoothnessTerm(n, i, (i+1)%light->numParameters(), r), NULL /*new ceres::HuberLoss(lightscale)*/, lightarr);
        }
    }
    for (int k = 0; k < light->numParameters(); k++, idx++) {
        problem->SetParameterLowerBound(lightarr, idx, 0);
    }
}

void addCeresCubemap(CubemapLight* light, ceres::Problem* problem, double* lightarr, int n, int idx)
{
    double r = light->getRegularization();
    if (r > 0) {
        vector<pair<int, int> > envmapAdjacencies;
        computeEnvmapAdjacencies(envmapAdjacencies, light->getCubemapResolution());
        //double lightscale = 10;
        for (int j = 0; j < envmapAdjacencies.size(); j++) {
            int a = envmapAdjacencies[j].first + idx;
            int b = envmapAdjacencies[j].second + idx;
            problem->AddResidualBlock(CreateSmoothnessTerm(n, a, b, r), NULL /*new ceres::HuberLoss(lightscale)*/, lightarr);
        }
    }
    for (int k = 0; k < light->numParameters(); k++, idx++) {
        problem->SetParameterLowerBound(lightarr, idx, 0);
    }
}
