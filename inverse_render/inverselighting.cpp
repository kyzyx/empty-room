#include "solver.h"
#include "ceres/ceres.h"

using namespace std;
using namespace ceres;


const double MINRELATIVEERROR = 0.01;

// Note: Error function as follows:
// Incident light L has three components: Direct (D), Indirect (I), Unknown (U)
// 1. Scale incident indirect light to fill up I and U components:
//      L_I' = L_I*(I+U)/I = L_I*(1-D)/(1-D-U)
// 2. Sum incident direct and indirect light for total incident light
//      L = L_I' + L_D
// 3. Absolute error for diffuse reflectance
//      err = L*p - B
// 4. Scale error by our confidence in the sample, which for now is 1-U
//      err *= (1-U)
// 5. Use relative error (with some tolerance to prevent exploding errors)
//      err /= (B + MINRELATIVEERROR)
struct IncidentFunctor {
    IncidentFunctor(SampleData& data, int channel)
        : d(data), ch(channel) { }

    template <typename T>
    bool operator() (
            const T* const materials,
            T* residual) const
    {
        T computed = materials[0]*T(d.netIncoming[ch]/(1-d.fractionUnknown));
        residual[0] = computed - T(d.radiosity[ch]);
        //residual[0] *= T(1-d.fractionUnknown)/T(d.radiosity[ch] + MINRELATIVEERROR);
        //residual[0] /= T(d.radiosity[ch] + MINRELATIVEERROR);
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
        T computed = T(0);
        for (int j = 0; j < nlights; j++) {
            computed += lights[j]*T(d.lightamount[j]);
        }
        computed += T(d.netIncoming[ch])*T((1-d.fractionDirect)/(1-d.fractionUnknown-d.fractionDirect));
        computed *= materials[0];
        residual[0] = computed - T(d.radiosity[ch]);
        //residual[0] *= T(1-d.fractionUnknown)/T(d.radiosity[ch] + MINRELATIVEERROR);
        //residual[0] /= T(d.radiosity[ch] + MINRELATIVEERROR);
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
        T computed = T(0);
        for (int j = 0; j < nlights; j++) {
            computed += params[1][j]*T(d.lightamount[j]);
        }
        computed += T(d.netIncoming[ch])*T((1-d.fractionDirect)/(1-d.fractionUnknown-d.fractionDirect));
        computed *= params[0][0];
        residual[0] = computed - T(d.radiosity[ch]);
        //residual[0] *= T(1-d.fractionUnknown)/T(d.radiosity[ch] + MINRELATIVEERROR);
        //residual[0] /= T(d.radiosity[ch] + MINRELATIVEERROR);
        return true;
    }

    SampleData d;
    int nlights;
    int ch;
};
struct SHRegularizerFunctor {
    SHRegularizerFunctor(int index, double lambda)
        : idx(index), l(std::sqrt(lambda)) { }

    template <typename T>
    bool operator() (
            const T* const lights,
            T* residual) const
    {
        residual[0] = lights[idx]*T(l);
        return true;
    }

    int idx;
    double l;
};

struct DynSHRegularizerFunctor {
    DynSHRegularizerFunctor(int index, double lambda)
        : idx(index), l(std::sqrt(lambda)) { }

    template <typename T>
    bool operator() (
            T const* const* params,
            T* residual) const
    {
        residual[0] = params[0][idx]*T(l);
        return true;
    }

    int idx;
    double l;
};

CostFunction* CreateSHRegularizer(int numlights, int i, double lambda) {
    switch (numlights) {
        case 0:
            return NULL;
        case 1:
            return (new ceres::AutoDiffCostFunction<SHRegularizerFunctor, 1, 1>(
                    new SHRegularizerFunctor(i, lambda)));
        case 2:
            return (new ceres::AutoDiffCostFunction<SHRegularizerFunctor, 1, 2>(
                    new SHRegularizerFunctor(i, lambda)));
        case 3:
            return (new ceres::AutoDiffCostFunction<SHRegularizerFunctor, 1, 3>(
                    new SHRegularizerFunctor(i, lambda)));
        case 4:
            return (new ceres::AutoDiffCostFunction<SHRegularizerFunctor, 1, 4>(
                    new SHRegularizerFunctor(i, lambda)));
        case 5:
            return (new ceres::AutoDiffCostFunction<SHRegularizerFunctor, 1, 5>(
                    new SHRegularizerFunctor(i, lambda)));
        case 6:
            return (new ceres::AutoDiffCostFunction<SHRegularizerFunctor, 1, 6>(
                    new SHRegularizerFunctor(i, lambda)));
        case 7:
            return (new ceres::AutoDiffCostFunction<SHRegularizerFunctor, 1, 7>(
                    new SHRegularizerFunctor(i, lambda)));
        case 8:
            return (new ceres::AutoDiffCostFunction<SHRegularizerFunctor, 1, 8>(
                    new SHRegularizerFunctor(i, lambda)));
        case 9:
            return (new ceres::AutoDiffCostFunction<SHRegularizerFunctor, 1, 9>(
                    new SHRegularizerFunctor(i, lambda)));
        default:
            if (numlights < 0) return NULL;
            ceres::DynamicAutoDiffCostFunction<DynSHRegularizerFunctor>* ret =
                new ceres::DynamicAutoDiffCostFunction<DynSHRegularizerFunctor>(
                        new DynSHRegularizerFunctor(i, lambda));
            ret->AddParameterBlock(numlights);
            ret->SetNumResiduals(1);
            return ret;
    }
}

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


void InverseRender::solve(vector<SampleData>& data, double reglambda) {
    if (reglambda > 0) reglambda /= NUM_SH_BANDS*NUM_SH_BANDS - 1;
    google::InitGoogleLogging("solveExposure()");
    double mat = 0.6;
    double* ls = new double[lights.size()];
    for (int i = 0; i < lights.size(); i++) ls[i] = lights[i](0);
    for (int z = 0; z < 3; z++) {
        ceres::Problem problem;
        for (int i = 0; i < data.size(); i++) {
            ceres::LossFunction* fn = NULL;
            if (lossfn == LOSS_CAUCHY) {
                fn = new ceres::CauchyLoss(scale);
            } else if (lossfn == LOSS_HUBER) {
                fn = new ceres::HuberLoss(scale);
            }
            if (lights.size()) {
                problem.AddResidualBlock(Create(data[i], lights.size(), z), fn, &mat, ls);
            } else {
                problem.AddResidualBlock(Create(data[i], lights.size(), z), fn, &mat);
            }
        }
        problem.SetParameterLowerBound(&mat, 0, 0);
        problem.SetParameterUpperBound(&mat, 0, 1);
        for (int i = 0; i < lights.size(); i++) {
            if (coeftype[i] != LIGHTTYPE_SH)
                problem.SetParameterLowerBound(ls, i, 0);
            else if (i > 0 && reglambda > 0)
                problem.AddResidualBlock(CreateSHRegularizer(lights.size(), i, reglambda), NULL, ls);
        }
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << "\n";
        wallMaterial(z) = mat;
        for (int i = 0; i < lights.size(); i++) lights[i](z) = ls[i];
    }
}
