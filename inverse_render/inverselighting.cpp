#include "solver.h"
#include "ceres/ceres.h"
#include "lighting/lighting_ceres.h"

using namespace std;
using namespace ceres;


const double MINRELATIVEERROR = 0.01;

void linearArrayFromLights(double* a, vector<Light*>& d) {
    int idx = 0;
    for (int i = 0; i < d.size(); i++) {
        for (int j = 0; j < d[i]->numParameters(); j++) {
            a[idx++] = d[i]->coef(j);
        }
    }
}
void lightsFromLinearArray(vector<Light*>& d, double* a) {
    int idx = 0;
    for (int i = 0; i < d.size(); i++) {
        for (int j = 0; j < d[i]->numParameters(); j++) {
            d[i]->coef(j) = a[idx++];
        }
    }
}

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
    IncidentFunctor(SampleData& data, int mid, int channel)
        : d(data), ch(channel), mat(mid) { }

    template <typename T>
    bool operator() (
            const T* const materials,
            T* residual) const
    {
        T computed = materials[mat]*T(d.netIncoming[ch]/(1-d.fractionUnknown));
        residual[0] = computed - T(d.radiosity[ch]);
        //residual[0] *= T(1-d.fractionUnknown)/T(d.radiosity[ch] + MINRELATIVEERROR);
        //residual[0] /= T(d.radiosity[ch] + MINRELATIVEERROR);
        return true;
    }

    int mat;
    SampleData d;
    int ch;
};

struct LightFunctor {
    LightFunctor(SampleData& data, int mid, int numlights, int channel)
        : d(data), nlights(numlights), ch(channel), mat(mid)
    {
        lightamount = new double[nlights];
        linearArrayFromLights(lightamount, d.lightamount);
    }

    template <typename T>
    bool operator() (
            const T* const materials,
            const T* const lights,
            T* residual) const
    {
        T computed = T(0);
        for (int j = 0; j < nlights; j++) {
            computed += lights[j]*T(lightamount[j]);
        }
        computed += T(d.netIncoming[ch])*T((1-d.fractionDirect)/(1-d.fractionUnknown-d.fractionDirect));
        computed *= materials[mat];
        residual[0] = computed - T(d.radiosity[ch]);
        //residual[0] *= T(1-d.fractionUnknown)/T(d.radiosity[ch] + MINRELATIVEERROR);
        //residual[0] /= T(d.radiosity[ch] + MINRELATIVEERROR);
        return true;
    }

    int mat;
    SampleData d;
    double* lightamount;
    int nlights;
    int ch;
};

struct DynLightFunctor {
    DynLightFunctor(SampleData& data, int mid, int numlights, int channel)
        : d(data), nlights(numlights), ch(channel), mat(mid)
    {
        lightamount = new double[nlights];
        linearArrayFromLights(lightamount, d.lightamount);
    }

    template <typename T>
    bool operator() (
            T const* const* params,
            T* residual) const
    {
        T computed = T(0);
        for (int j = 0; j < nlights; j++) {
            computed += params[1][j]*T(lightamount[j]);
        }
        computed += T(d.netIncoming[ch])*T((1-d.fractionDirect)/(1-d.fractionUnknown-d.fractionDirect));
        computed *= params[0][mat];
        residual[0] = computed - T(d.radiosity[ch]);
        //residual[0] *= T(1-d.fractionUnknown)/T(d.radiosity[ch] + MINRELATIVEERROR);
        //residual[0] /= T(d.radiosity[ch] + MINRELATIVEERROR);
        return true;
    }

    int mat;
    SampleData d;
    int nlights;
    int ch;
    double* lightamount;
};

CostFunction* Create(SampleData& sd, int mid, int nummats, int numlights, int ch) {
    ceres::DynamicAutoDiffCostFunction<DynLightFunctor>* ret =
        new ceres::DynamicAutoDiffCostFunction<DynLightFunctor>(
                new DynLightFunctor(sd, mid, numlights, ch));
    ret->AddParameterBlock(nummats);
    ret->AddParameterBlock(numlights);
    ret->SetNumResiduals(1);
    return ret;
}


void InverseRender::solve(vector<SampleData>& data, double reglambda) {
    google::InitGoogleLogging("solveExposure()");
    int nummats = 0;
    for (int i = 0; i < data.size(); i++) {
        int materialid = mesh->getLabel(data[i].vertexid, MeshManager::TYPE_CHANNEL);
        if (materialid > 0) nummats = std::max(nummats, materialid);
    }
    materials.resize(nummats);
    int numlights = 0;
    for (int i = 0; i < lights[0].size(); i++) {
        numlights += lights[0][i]->numParameters();
    }
    double* ls = new double[numlights];
    linearArrayFromLights(ls, lights[0]);
    double* mat = new double[nummats];
    for (int i = 0; i < nummats; i++) mat[i] = 0.6;
    for (int z = 0; z < 3; z++) {
        ceres::Problem problem;
        for (int i = 0; i < data.size(); i++) {
            //if (data[i].netIncoming[z] > data[i].radiosity[z]) continue;
            int materialid = mesh->getLabel(data[i].vertexid, MeshManager::TYPE_CHANNEL);
            if (materialid > 0) materialid--;
            else continue;
            ceres::LossFunction* fn = NULL;
            if (lossfn == LOSS_CAUCHY) {
                fn = new ceres::CauchyLoss(scale);
            } else if (lossfn == LOSS_HUBER) {
                fn = new ceres::HuberLoss(scale);
            }
            problem.AddResidualBlock(Create(data[i], materialid, nummats, numlights, z), fn, mat, ls);
        }
        for (int i = 0; i < nummats; i++) {
            problem.SetParameterLowerBound(mat, i, 0);
            problem.SetParameterUpperBound(mat, i, 1);
        }
        int curridx = 0;
        for (int i = 0; i < lights[z].size(); i++) {
            addCeres(lights[z][i], &problem, ls, numlights, curridx);
            curridx += lights[z][i]->numParameters();
        }
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << "\n";
        wallMaterial(z) = mat[0];
        for (int i = 0; i < nummats; i++) {
            materials[i](z) = mat[i];
        }
        lightsFromLinearArray(lights[z], ls);
    }
}
