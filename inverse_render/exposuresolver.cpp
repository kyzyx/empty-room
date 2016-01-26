#include "exposuresolver.h"
#include "ceres/ceres.h"

using namespace std;
using namespace ceres;

struct OverexposedFunctor {
    OverexposedFunctor(float lambda) : l(lambda) {}
    template <typename T>
    bool operator()(
            const T* const camera,
            const T* const point,
            T* residual) const
    {
        residual[0] = camera[0]*point[0] - T(1.f);
        if (camera[0]*point[0] > T(1.f)) residual[0] *= T(l);
        return true;
    }

    float l;
};

struct CostFunctor {
    CostFunctor(float pixelvalue) : v(pixelvalue) {}
    template <typename T>
    bool operator()(
            const T* const camera,
            const T* const point,
            T* residual) const
    {
        residual[0] = camera[0]*point[0] - T(v);
        return true;
    }
    float v;
};

const float lambda = 0.f;

CostFunction* Create(float pixelvalue) {
    if (pixelvalue >= 1.f) {
        return (new ceres::AutoDiffCostFunction<OverexposedFunctor, 1, 1, 1>(
                    new OverexposedFunctor(lambda)));
    } else {
        return (new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1>(
                    new CostFunctor(pixelvalue)));
    }
}


double solveExposure(
        MeshManager* manager,
        int n,
        double* exposures,
        double* radiances,
        std::vector<int>& indices,
        int lossfn,
        float scale)
{
    google::InitGoogleLogging("solveExposure()");
    if (indices.empty()) {
        for (int i = 0; i < manager->NVertices(); i++) {
            indices.push_back(i);
        }
    }
    double cost = 0;
    for (int ch = 0; ch < 3; ch++) {
        double* exps = exposures + ch*n;
        if (ch) {
            memcpy(exps,
                   exposures + (ch-1)*n,
                   sizeof(double)*n);
        }
        ceres::Problem problem;
        for (int i = 0; i < indices.size(); i++) {
            for (int j = 0; j < manager->getVertexSampleCount(indices[i]); j++) {
                Sample s = manager->getSample(indices[i], j);
                ceres::CostFunction* costfn;
                if (ch == 0) costfn = Create(s.r);
                else if (ch == 1) costfn = Create(s.g);
                else if (ch == 2) costfn = Create(s.b);
                if (lossfn == LOSS_CAUCHY) {
                    problem.AddResidualBlock(
                            costfn, new ceres::CauchyLoss(scale), &exps[s.id], &radiances[i]);
                } else if (lossfn == LOSS_HUBER) {
                    problem.AddResidualBlock(
                            costfn, new ceres::HuberLoss(scale), &exps[s.id], &radiances[i]);
                } else {
                    problem.AddResidualBlock(
                            costfn, NULL, &exps[s.id], &radiances[i]);
                }
                problem.SetParameterLowerBound(&exps[s.id], 0, 0);
            }
            problem.SetParameterLowerBound(&radiances[i], 0, 0);
        }
        problem.SetParameterBlockConstant(&exps[0]);
        ceres::Solver::Options options;
        //options.linear_solver_type = ceres::DENSE_SCHUR;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.minimizer_progress_to_stdout = true;
        options.use_inner_iterations = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        //std::cout << summary.FullReport() << "\n";
        cost = max(cost, summary.final_cost);
    }
    return cost;
}
