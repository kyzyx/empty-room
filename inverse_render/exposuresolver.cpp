#include "exposuresolver.h"
#include "ceres/ceres.h"

using namespace std;
using namespace ceres;

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
    static CostFunction* Create(float pixelvalue) {
        return (new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1>(
                    new CostFunctor(pixelvalue)));
    }

    float v;
};

double solveExposure(
        MeshManager* manager,
        int n,
        double* exposures,
        double* radiances,
        std::vector<int>& indices)
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
                if (ch == 0) costfn = CostFunctor::Create(s.r);
                else if (ch == 1) costfn = CostFunctor::Create(s.g);
                else if (ch == 2) costfn = CostFunctor::Create(s.b);
                problem.AddResidualBlock(
                    costfn, NULL, &exps[s.id], &radiances[i]);
            }
        }
        problem.SetParameterBlockConstant(&exps[0]);
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        //options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        //std::cout << summary.FullReport() << "\n";
        cost = max(cost, summary.final_cost);
    }
    return cost;
}
