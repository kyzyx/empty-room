#include "lighting_ceres.h"
#include "ceres/ceres.h"
#include "sh.h"
#include "cubemap.h"

#include "ceres_reg.h"

using namespace std;

void addCeres(const Light* light, ceres::Problem* problem, double* lightarr, int n, int idx)
{
    if (light->typeId() & LIGHTTYPE_RGB) {
        RGBLight* rgbl = (RGBLight*) light;
        int np = rgbl->getLight(0)->numParameters();
        for (int i = 0; i < 3; i++) {
            addCeres(rgbl->getLight(i), problem, lightarr, n, idx + i*np);
        }
    } else if (light->typeId() & LIGHTTYPE_IRGB) {
        IRGBLight* irgbl = (IRGBLight*) light;
        for (int i = 0; i < 3; i++) {
            problem->SetParameterLowerBound(lightarr, idx+i, 0);
        }
        problem->AddResidualBlock(CreateIRGBTerm(idx, n, 1e8), NULL, lightarr);
        addCeres(irgbl->getLight(), problem, lightarr, n, idx+3);
    } else {
        switch (light->typeId()) {
            case LIGHTTYPE_POINT:
                addCeresSymmetricPoint((SymmetricPointLight*) light, problem, lightarr, n, idx);
                break;
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
}

void setNonnegative(const Light* light, ceres::Problem* problem, double* lightarr, int n, int idx)
{
    for (int k = 0; k < light->numParameters(); k++, idx++) {
        problem->SetParameterLowerBound(lightarr, idx, 0);
    }
}

void addCeresArea(const AreaLight* light, ceres::Problem* problem, double* lightarr, int n, int idx)
{
    setNonnegative(light, problem, lightarr, n, idx);
}

void addCeresSH(const SHLight* light, ceres::Problem* problem, double* lightarr, int n, int idx)
{
    double r = light->getRegularization();
    if (r > 0) {
        for (int i = 1; i < light->numParameters(); i++) {
            problem->AddResidualBlock(CreateSHRegularizer(n, i+idx, r), NULL, lightarr);
        }
    }
}

void addCeresSymmetricPoint(const SymmetricPointLight* light, ceres::Problem* problem, double* lightarr, int n, int idx)
{
    double r = light->getRegularization();
    if (r > 0) {
        for (int i = 0; i < light->numParameters()-1; i++) {
            problem->AddResidualBlock(CreateSmoothnessTerm(n, i, i+1, r), NULL /*new ceres::HuberLoss(lightscale)*/, lightarr);
        }
    }
    setNonnegative(light, problem, lightarr, n, idx);
}

void addCeresLine(const LineLight* light, ceres::Problem* problem, double* lightarr, int n, int idx)
{
    double r = light->getRegularization();
    if (r > 0) {
        int lim = light->numParameters();
        if (light->isSymmetric()) lim /= 2;
        for (int i = 0; i < lim; i++) {
            problem->AddResidualBlock(CreateSmoothnessTerm(n, i, (i+1)%light->numParameters(), r), NULL /*new ceres::HuberLoss(lightscale)*/, lightarr);
        }
    }
    setNonnegative(light, problem, lightarr, n, idx);
}

void addCeresCubemap(const CubemapLight* light, ceres::Problem* problem, double* lightarr, int n, int idx)
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
    setNonnegative(light, problem, lightarr, n, idx);
}
