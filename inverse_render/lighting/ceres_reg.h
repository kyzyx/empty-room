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

struct IRGBFunctor {
    IRGBFunctor(int startidx, double lambda)
        : idx(startidx), l(std::sqrt(lambda)) { }

    template <typename T>
    bool operator() (
            T const* const* params,
            T* residual) const
    {
        residual[0] = T(0);
        for (int i = 0; i < 3; i++) residual[0] += params[0][idx+i]*params[0][idx+i];
        residual[0] = T(l)*(sqrt(residual[0]) - T(1));
        return true;
    }

    int idx;
    double l;
};

ceres::CostFunction* CreateIRGBTerm(int i, int numlights, double lambda) {
    ceres::DynamicAutoDiffCostFunction<IRGBFunctor>* ret =
        new ceres::DynamicAutoDiffCostFunction<IRGBFunctor>(
                new IRGBFunctor(i, lambda));
    ret->AddParameterBlock(numlights);
    ret->SetNumResiduals(1);
    return ret;
}
