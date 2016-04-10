#include <cmath>

struct SmoothnessFunctor {
    SmoothnessFunctor(int a, int b, double lambda)
        : i1(a), i2(b), l(std::sqrt(lambda)) { }

    template <typename T>
    bool operator() (
            const T* const lights,
            T* residual) const
    {
        residual[0] = T(l)*(lights[i1] - lights[i2]);
        return true;
    }

    int i1, i2;
    double l;
};

struct DynSmoothnessFunctor {
    DynSmoothnessFunctor(int a, int b, double lambda)
        : i1(a), i2(b), l(std::sqrt(lambda)) { }

    template <typename T>
    bool operator() (
            T const* const* params,
            T* residual) const
    {
        residual[0] = (params[0][i1] - params[0][i2])*T(l);
        return true;
    }

    int i1, i2;
    double l;
};

ceres::CostFunction* CreateSmoothnessTerm(int numlights, int a, int b, double lambda) {
    switch (numlights) {
        case 0:
            return NULL;
        case 1:
            return (new ceres::AutoDiffCostFunction<SmoothnessFunctor, 1, 1>(
                    new SmoothnessFunctor(a, b, lambda)));
        case 2:
            return (new ceres::AutoDiffCostFunction<SmoothnessFunctor, 1, 2>(
                    new SmoothnessFunctor(a, b, lambda)));
        case 3:
            return (new ceres::AutoDiffCostFunction<SmoothnessFunctor, 1, 3>(
                    new SmoothnessFunctor(a, b, lambda)));
        case 4:
            return (new ceres::AutoDiffCostFunction<SmoothnessFunctor, 1, 4>(
                    new SmoothnessFunctor(a, b, lambda)));
        case 5:
            return (new ceres::AutoDiffCostFunction<SmoothnessFunctor, 1, 5>(
                    new SmoothnessFunctor(a, b, lambda)));
        case 6:
            return (new ceres::AutoDiffCostFunction<SmoothnessFunctor, 1, 6>(
                    new SmoothnessFunctor(a, b, lambda)));
        case 7:
            return (new ceres::AutoDiffCostFunction<SmoothnessFunctor, 1, 7>(
                    new SmoothnessFunctor(a, b, lambda)));
        case 8:
            return (new ceres::AutoDiffCostFunction<SmoothnessFunctor, 1, 8>(
                    new SmoothnessFunctor(a, b, lambda)));
        case 9:
            return (new ceres::AutoDiffCostFunction<SmoothnessFunctor, 1, 9>(
                    new SmoothnessFunctor(a, b, lambda)));
        default:
            if (numlights < 0) return NULL;
            ceres::DynamicAutoDiffCostFunction<DynSmoothnessFunctor>* ret =
                new ceres::DynamicAutoDiffCostFunction<DynSmoothnessFunctor>(
                        new DynSmoothnessFunctor(a, b, lambda));
            ret->AddParameterBlock(numlights);
            ret->SetNumResiduals(1);
            return ret;
    }
}

// ----------------------------------------------------------------------------

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

ceres::CostFunction* CreateSHRegularizer(int numlights, int i, double lambda) {
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

// ----------------------------------------------------------------------------
struct YIQRGBFunctor {
    YIQRGBFunctor(int numlights, int startidx, double lambda)
        : n(numlights), s(startidx), l(lambda) {}
    template <typename T>
    bool operator() (
            T const* const* params,
            T* residual) const
    {
        for (int i = 0; i < 3; i++) residual[i] = T(0);
        for (int i = 0; i < n; i++) {
            T a = params[0][s+i+2] + T(0.956)*params[0][s] + T(0.621)*params[0][s+1];
            if (a < T(0)) residual[0] += a*T(l);
            T b = params[0][s+i+2] + T(-0.272)*params[0][s] + T(-0.647)*params[0][s+1];
            if (b < T(0)) residual[1] += a*T(l);
            T c = params[0][s+i+2] + T(-1.107)*params[0][s] + T(1.705)*params[0][s+1];
            if (c < T(0)) residual[2] += a*T(l);
        }
        return true;
    }
    int n, s;
    double l;
};
ceres::CostFunction* CreateYIQRGB(int numlights, int numparams, int i, double lambda) {
    ceres::DynamicAutoDiffCostFunction<YIQRGBFunctor>* ret =
        new ceres::DynamicAutoDiffCostFunction<YIQRGBFunctor>(
                new YIQRGBFunctor(numparams, i, lambda));
    ret->AddParameterBlock(numlights);
    ret->SetNumResiduals(3);
    return ret;
}
