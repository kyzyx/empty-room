#include "solver.h"
#include "ceres/ceres.h"
#define MAXAREALIGHTS 8

using namespace std;
using namespace ceres;

// Params: robustness, robustness scale

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
        return true;
    }

    SampleData d;
    int ch;
};

struct AreaLightFunctor {
    AreaLightFunctor(SampleData& data, int numarealights, int channel)
        : d(data), narealights(numarealights), ch(channel) { }

    template <typename T>
    bool operator() (
            const T* const materials,
            const T* const arealights,
            T* residual) const
    {
        T computed = T(d.netIncoming[ch]);
        for (int j = 0; j < narealights; j++) {
            computed += arealights[j]*T(d.lightamount[j]);
        }
        computed *= materials[0]*T(1-d.fractionUnknown);
        residual[0] = computed - T(d.radiosity[ch]);
        return true;
    }

    SampleData d;
    int narealights;
    int ch;
};

CostFunction* Create(SampleData& sd, int numarealights, int ch) {
    switch (numarealights) {
        case 0:
            return (new ceres::AutoDiffCostFunction<IncidentFunctor, 1, 1>(
                        new IncidentFunctor(sd, ch)));
        case 1:
            return (new ceres::AutoDiffCostFunction<AreaLightFunctor, 1, 1, 1>(
                        new AreaLightFunctor(sd, numarealights, ch)));
        case 2:
            return (new ceres::AutoDiffCostFunction<AreaLightFunctor, 1, 2, 1>(
                        new AreaLightFunctor(sd, numarealights, ch)));
        case 3:
            return (new ceres::AutoDiffCostFunction<AreaLightFunctor, 1, 3, 1>(
                        new AreaLightFunctor(sd, numarealights, ch)));
        case 4:
            return (new ceres::AutoDiffCostFunction<AreaLightFunctor, 1, 4, 1>(
                        new AreaLightFunctor(sd, numarealights, ch)));
        case 5:
            return (new ceres::AutoDiffCostFunction<AreaLightFunctor, 1, 5, 1>(
                        new AreaLightFunctor(sd, numarealights, ch)));
        case 6:
            return (new ceres::AutoDiffCostFunction<AreaLightFunctor, 1, 6, 1>(
                        new AreaLightFunctor(sd, numarealights, ch)));
        default:
            printf("Error! Unsupported number of area lights %d\n", numarealights);
    }
}

void InverseRender::solve(vector<SampleData>& data) {
    google::InitGoogleLogging("solveExposure()");
    double mat = 0.6;
    double* arealights = new double[numlights];
    for (int i = 0; i < numlights; i++) arealights[i] = 10;
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
                problem.AddResidualBlock(Create(data[i], numlights, z), fn, &mat, arealights);
            } else {
                problem.AddResidualBlock(Create(data[i], numlights, z), fn, &mat);
            }
            // TODO: Robust norms
        }
        problem.SetParameterLowerBound(&mat, 0, 0);
        problem.SetParameterUpperBound(&mat, 0, 1);
        for (int i = 0; i < numlights; i++) problem.SetParameterLowerBound(arealights, i, 0);
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << "\n";
        wallMaterial(z) = mat;
        for (int i = 0; i < numlights; i++) lights[i](z) = arealights[i];
    }
}
