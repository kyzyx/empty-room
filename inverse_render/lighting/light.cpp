#include "light.h"
#include "sh.h"
#include "cubemap.h"
#include "ceres/ceres.h"

using namespace std;

#include "ceres_reg.h"

double Light::lightContribution(double* it) const {
    double ret = 0;
    for (auto vit = v.begin(); vit != v.end(); ++vit, ++it) {
        ret += *vit * *it;
    }
    return ret;
}

double Light::lightContribution(vector<double>::const_iterator it) const {
    double ret = 0;
    for (auto vit = v.begin(); vit != v.end(); ++vit, ++it) {
        ret += *vit * *it;
    }
    return ret;
}

double Light::lightContribution(Light* l) const {
    return l->lightContribution(v.begin());
}

void AreaLight::addCeres(ceres::Problem* problem, double* lightarr, int n, int idx) {
    problem->SetParameterLowerBound(lightarr, idx, 0);
}

void SHEnvironmentLight::addCeres(ceres::Problem* problem, double* lightarr, int n, int idx) {
    if (reglambda > 0) {
        for (int i = 1; i < numParameters(); i++) {
            problem->AddResidualBlock(CreateSHRegularizer(n, i+idx, reglambda), NULL, lightarr);
        }
    }
}

void SHEnvironmentLight::addLightFF(
        double px, double py, double pz,
        double dx, double dy, double dz,
        double weight)
{
    int idx = 0;
    for (int band = 0; band < numbands; band++) {
        for (int m = -band; m <= band; m++) {
            v[idx++] += weight*SH(band, m, dx, dy, dz);
        }
    }
}

void CubemapEnvironmentLight::addCeres(ceres::Problem* problem, double* lightarr, int n, int idx) {
    if (reglambda > 0) {
        vector<pair<int, int> > envmapAdjacencies;
        computeEnvmapAdjacencies(envmapAdjacencies, res);
        //double lightscale = 10;
        for (int j = 0; j < envmapAdjacencies.size(); j++) {
            int a = envmapAdjacencies[j].first + idx;
            int b = envmapAdjacencies[j].second + idx;
            problem->AddResidualBlock(CreateSmoothnessTerm(n, a, b, reglambda), NULL /*new ceres::HuberLoss(lightscale)*/, lightarr);
        }
    }
    for (int k = 0; k < numParameters(); k++, idx++) {
        problem->SetParameterLowerBound(lightarr, idx, 0);
    }
}

void CubemapEnvironmentLight::addLightFF(
        double px, double py, double pz,
        double dx, double dy, double dz,
        double weight)
{
    int envmapidx = getEnvmapCell(dx, dy, dz, res);
    v[envmapidx] += weight;
}

Light* NewLightFromLightType(int type) {
    switch (type) {
        case LIGHTTYPE_SH: return new SHEnvironmentLight;
        case LIGHTTYPE_ENVMAP: return new CubemapEnvironmentLight;
        case LIGHTTYPE_AREA: return new AreaLight;
        default: return NULL;
    }
}
