#include "solver.h"
#include "ceres/ceres.h"

using namespace std;
using namespace ceres;


const double MINRELATIVEERROR = 0.01;

struct IncidentFunctor {
    IncidentFunctor(SampleData& data, int channel)
        : d(data), ch(channel) { }

    template <typename T>
    bool operator() (
            const T* const materials,
            T* residual) const
    {
        T computed = materials[0]*T(d.netIncoming[ch]*(1-d.fractionUnknown));
        residual[0] = computed - T(d.radiosity[ch]);
        residual[0] /= T(d.radiosity[ch] + MINRELATIVEERROR);
        return true;
    }

    SampleData d;
    int ch;
};

struct LightFunctor {
    LightFunctor(SampleData& data, int numlights, int channel)
        : d(data), nlights(numlights), ch(channel) { }

    template <typename T>
    bool operator() (
            const T* const materials,
            const T* const lights,
            T* residual) const
    {
        T computed = T(d.netIncoming[ch]);
        for (int j = 0; j < nlights; j++) {
            computed += lights[j]*T(d.lightamount[j]);
        }
        computed *= materials[0]*T(1-d.fractionUnknown);
        residual[0] = computed - T(d.radiosity[ch]);
        residual[0] /= T(d.radiosity[ch] + MINRELATIVEERROR);
        return true;
    }

    SampleData d;
    int nlights;
    int ch;
};

struct DynLightFunctor {
    DynLightFunctor(SampleData& data, int numlights, int channel)
        : d(data), nlights(numlights), ch(channel) { }

    template <typename T>
    bool operator() (
            T const* const* params,
            T* residual) const
    {
        T computed = T(d.netIncoming[ch]);
        for (int j = 0; j < nlights; j++) {
            computed += params[1][j]*T(d.lightamount[j]);
        }
        computed *= params[0][0]*T(1-d.fractionUnknown);
        residual[0] = computed - T(d.radiosity[ch]);
        residual[0] /= T(d.radiosity[ch] + MINRELATIVEERROR);
        return true;
    }

    SampleData d;
    int nlights;
    int ch;
};

CostFunction* Create(SampleData& sd, int numlights, int ch) {
    switch (numlights) {
        case 0:
            return (new ceres::AutoDiffCostFunction<IncidentFunctor, 1, 1>(
                        new IncidentFunctor(sd, ch)));
        case 1:
            return (new ceres::AutoDiffCostFunction<LightFunctor, 1, 1, 1>(
                        new LightFunctor(sd, numlights, ch)));
        case 2:
            return (new ceres::AutoDiffCostFunction<LightFunctor, 1, 1, 2>(
                        new LightFunctor(sd, numlights, ch)));
        case 3:
            return (new ceres::AutoDiffCostFunction<LightFunctor, 1, 1, 3>(
                        new LightFunctor(sd, numlights, ch)));
        case 4:
            return (new ceres::AutoDiffCostFunction<LightFunctor, 1, 1, 4>(
                        new LightFunctor(sd, numlights, ch)));
        case 5:
            return (new ceres::AutoDiffCostFunction<LightFunctor, 1, 1, 5>(
                        new LightFunctor(sd, numlights, ch)));
        case 6:
            return (new ceres::AutoDiffCostFunction<LightFunctor, 1, 1, 6>(
                        new LightFunctor(sd, numlights, ch)));
        case 7:
            return (new ceres::AutoDiffCostFunction<LightFunctor, 1, 1, 7>(
                        new LightFunctor(sd, numlights, ch)));
        case 8:
            return (new ceres::AutoDiffCostFunction<LightFunctor, 1, 1, 8>(
                        new LightFunctor(sd, numlights, ch)));
        case 9:
            return (new ceres::AutoDiffCostFunction<LightFunctor, 1, 1, 9>(
                        new LightFunctor(sd, numlights, ch)));
        default:
            if (numlights < 0) return NULL;
            ceres::DynamicAutoDiffCostFunction<DynLightFunctor>* ret =
                new ceres::DynamicAutoDiffCostFunction<DynLightFunctor>(
                        new DynLightFunctor(sd, numlights, ch));
            ret->AddParameterBlock(1);
            ret->AddParameterBlock(numlights);
            ret->SetNumResiduals(1);
            return ret;
    }
}

void InverseRender::solve(vector<SampleData>& data) {
    google::InitGoogleLogging("solveExposure()");
    double mat = 0.6;
    double* ls = new double[numlights];
    for (int i = 0; i < numlights; i++) ls[i] = 10;
    for (int z = 0; z < 3; z++) {
        ceres::Problem problem;
        for (int i = 0; i < data.size(); i++) {
            ceres::LossFunction* fn = NULL;
            if (lossfn == LOSS_CAUCHY) {
                fn = new ceres::CauchyLoss(scale);
            } else if (lossfn == LOSS_HUBER) {
                fn = new ceres::HuberLoss(scale);
            }
            if (numlights) {
                problem.AddResidualBlock(Create(data[i], numlights, z), fn, &mat, ls);
            } else {
                problem.AddResidualBlock(Create(data[i], numlights, z), fn, &mat);
            }
        }
        problem.SetParameterLowerBound(&mat, 0, 0);
        problem.SetParameterUpperBound(&mat, 0, 1);
        for (int i = 0; i < numlights; i++) problem.SetParameterLowerBound(ls, i, 0);
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << "\n";
        wallMaterial(z) = mat;
        for (int i = 0; i < numlights; i++) lights[i](z) = ls[i];
    }
}
