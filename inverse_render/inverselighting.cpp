#include "solver.h"
#include "ceres/ceres.h"
#include "lighting/lighting_ceres.h"

using namespace std;
using namespace ceres;
using namespace Eigen;


const double obslambda = 0.00001;
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
struct DynLightFunctor {
    DynLightFunctor(SampleData& data, vector<Light*>& lights, int mid, double weight)
        : d(data), mat(mid), lightintensities(lights), wt(weight)
    {
    }

    template <typename T>
    bool operator() (
            T const* const* params,
            T* residual) const
    {
        for (int ch = 0; ch < 3; ch++) {
            residual[ch] = T(d.netIncoming[ch])*T(reweightIncoming(d));
        }
        int z = 0;
        for (int i = 0; i < d.lightamount.size(); i++) {
            lightContribution(lightintensities[i], residual, d.lightamount[i], params[1] + z);
            z += lightintensities[i]->numParameters();
        }
        for (int ch = 0; ch < 3; ch++) {
            residual[ch] *= params[0][mat*3+ch];
            residual[ch] -= T(d.radiosity[ch]);
            residual[ch] *= T(wt);
            //residual[ch] /= T(sqrt(d.radiosity[ch]) + MINRELATIVEERROR);
            //residual[ch] *= T(1-d.fractionUnknown)/T(d.radiosity[ch] + MINRELATIVEERROR);
            //residual[ch] /= T(d.radiosity[ch] + MINRELATIVEERROR);
        }
        return true;
    }

    int mat;
    const SampleData& d;
    vector<Light*>& lightintensities;
    double wt;
};
struct SingleChannelLightFunctor {
    SingleChannelLightFunctor(SampleData& data, vector<Light*>& lights, int mid, int channel, double weight)
        : d(data), mat(mid), ch(channel), wt(weight)
    {
        for (int i = 0; i < lights.size(); i++) {
            RGBLight* rgbl = static_cast<RGBLight*>(lights[i]);
            lightintensities.push_back(rgbl->getLight(ch));
        }
    }

    template <typename T>
    bool operator() (
            T const* const* params,
            T* residual) const
    {
        residual[0] = T(d.netIncoming[ch])*T(reweightIncoming(d));
        int z = 0;
        for (int i = 0; i < d.lightamount.size(); i++) {
            lightContribution(lightintensities[i], residual, d.lightamount[i], params[1]+z);
            z += lightintensities[i]->numParameters();
        }
        residual[0] *= params[0][mat];
        residual[0] -= T(d.radiosity[ch]);
        residual[0] *= T(wt);
        //residual[ch] /= T(sqrt(d.radiosity[ch]) + MINRELATIVEERROR);
        //residual[ch] *= T(1-d.fractionUnknown)/T(d.radiosity[ch] + MINRELATIVEERROR);
        //residual[ch] /= T(d.radiosity[ch] + MINRELATIVEERROR);
        return true;
    }

    int ch;
    int mat;
    const SampleData& d;
    double wt;
    vector<Light*> lightintensities;
};

CostFunction* Create(SampleData& sd, int mid, int nummats, int numlights, vector<Light*>& lightintensities, double weight) {
    ceres::DynamicAutoDiffCostFunction<DynLightFunctor>* ret =
        new ceres::DynamicAutoDiffCostFunction<DynLightFunctor>(
                new DynLightFunctor(sd, lightintensities, mid, weight));
    ret->AddParameterBlock(nummats);
    ret->AddParameterBlock(numlights);
    ret->SetNumResiduals(3);
    return ret;
}
CostFunction* CreateSingleChannel(SampleData& sd, int mid, int nummats, int numlights, vector<Light*>& lightintensities, int channel, double weight) {
    ceres::DynamicAutoDiffCostFunction<SingleChannelLightFunctor>* ret =
        new ceres::DynamicAutoDiffCostFunction<SingleChannelLightFunctor>(
                new SingleChannelLightFunctor(sd, lightintensities, mid, channel, weight));
    ret->AddParameterBlock(nummats);
    ret->AddParameterBlock(numlights);
    ret->SetNumResiduals(1);
    return ret;
}


void InverseRender::solve(vector<SampleData>& data, double reglambda, bool reweight) {
    bool issingle = true;
    for (int i = 0; i < lightintensities.size(); i++) {
        if (lightintensities[i]->typeId() & LIGHTTYPE_RGB) {
        } else {
            issingle = false;
        }
    }
    bool lightsonly = false;
    if (issingle) {
        solveSingleChannel(data, reglambda, reweight);
        return;
    }
    google::InitGoogleLogging("solveExposure()");
    int nummats = 0;
    for (int i = 0; i < data.size(); i++) {
        int materialid = mesh->getLabel(data[i].vertexid, MeshManager::TYPE_CHANNEL);
        if (materialid > 0) nummats = std::max(nummats, materialid);
    }
    materials.resize(nummats);
    nummats *= 3;
    int numlights = 0;
    for (int i = 0; i < lightintensities.size(); i++) {
        numlights += lightintensities[i]->numParameters();
    }
    double* ls = new double[numlights];
    linearArrayFromLights(ls, lightintensities);
    double* mat = new double[nummats];
    for (int i = 0; i < materials.size(); i++) {
        for (int j = 0; j < 3; j++)
            mat[3*i+j] = materials[i][j];
    }
    ceres::Problem problem;
    for (int i = 0; i < data.size(); i++) {
        //if (data[i].netIncoming[z] > data[i].radiosity[z]) continue;
        int materialid = mesh->getLabel(data[i].vertexid, MeshManager::TYPE_CHANNEL);
        if (materialid > 0) materialid--;
        else continue;
        double meanexitant = data[i].radiosity.r + data[i].radiosity.g + data[i].radiosity.b;
        double meanincident = data[i].netIncoming.r + data[i].netIncoming.g + data[i].netIncoming.b;
        meanincident *= reweightIncoming(data[i]);
        double weight = 1;
        double thr = 0.8;
        double weightscale = 15;
        if (!reweight && meanexitant > thr*meanincident) {
            weight = std::exp(weightscale*(1 - meanincident/(thr*meanexitant)));
        }
        ceres::LossFunction* fn = NULL;
        if (lossfn == LOSS_CAUCHY) {
            fn = new ceres::CauchyLoss(scale);
        } else if (lossfn == LOSS_HUBER) {
            fn = new ceres::HuberLoss(scale);
        }
        problem.AddResidualBlock(Create(data[i], materialid, nummats, numlights, lightintensities, weight), fn, mat, ls);
    }
    if (lightsonly) {
        problem.SetParameterBlockConstant(mat);
    } else {
        for (int i = 0; i < nummats; i++) {
            problem.SetParameterLowerBound(mat, i, 0);
            problem.SetParameterUpperBound(mat, i, 1);
        }
    }
    int curridx = 0;
    for (int i = 0; i < lightintensities.size(); i++) {
        if (lightintensities[i]->typeId() & LIGHTTYPE_ENVMAP) {
            vector<Vector3f> directions;
            vector<Vector3f> colors;
            vector<double> weights;
            for (int j = 0; j < mesh->NVertices(); j++) {
                int lightid = LIGHTID(mesh->getLabel(j, MeshManager::LABEL_CHANNEL));
                if (lightid == i) {
                    for (int k = 0; k < mesh->getVertexSampleCount(j); k++) {
                        Sample s = mesh->getSample(j, k);
                        colors.push_back(Vector3f(s.r, s.g, s.b));
                        directions.push_back(Vector3f(s.x, s.y, s.z));
                        weights.push_back(s.dA*s.confidence);
                    }
                }
            }
            addObservedConstraints(lightintensities[i], &problem, ls, numlights, obslambda, directions, colors, weights, -1, curridx);
        }

        addCeres(lightintensities[i], &problem, ls, numlights, curridx);
        curridx += lightintensities[i]->numParameters();
    }
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    for (int z = 0; z < 3; z++) {
        wallMaterial(z) = mat[z];
        for (int i = 0; i < nummats/3; i++) {
            materials[i](z) = mat[i*3+z];
        }
    }
    lightsFromLinearArray(lightintensities, ls);
}

void InverseRender::solveSingleChannel(vector<SampleData>& data, double reglambda, bool reweight) {
    google::InitGoogleLogging("solveExposure()");
    int nummats = 0;
    for (int i = 0; i < data.size(); i++) {
        int materialid = mesh->getLabel(data[i].vertexid, MeshManager::TYPE_CHANNEL);
        if (materialid > 0) nummats = std::max(nummats, materialid);
    }
    bool lightsonly = false;
    materials.resize(nummats);
    int numlights = 0;
    for (int i = 0; i < lightintensities.size(); i++) {
        if (lightintensities[i]->typeId() & LIGHTTYPE_RGB) {
            RGBLight* l = static_cast<RGBLight*>(lightintensities[i]);
            numlights += l->getLight(0)->numParameters();
        } else {
            return;
        }
    }
    double* ls = new double[numlights];
    for (int i = 0; i < numlights; i++) {
        ls[i] = 0;
    }
    double* mat = new double[nummats];
    for (int ch = 0; ch < 3; ch++) {
        for (int i = 0; i < nummats; i++) {
            mat[i] = materials[i][ch];
        }
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
            double meanexitant = data[i].radiosity.r + data[i].radiosity.g + data[i].radiosity.b;
            double meanincident = data[i].netIncoming.r + data[i].netIncoming.g + data[i].netIncoming.b;
            meanincident *= reweightIncoming(data[i]);
            double weight = 1;
            double thr = 0.8;
            double weightscale = 15;
            if (!reweight && meanexitant > thr*meanincident) {
                weight = std::exp(weightscale*(1 - meanincident/(thr*meanexitant)));
            }
            problem.AddResidualBlock(CreateSingleChannel(data[i], materialid, nummats, numlights, lightintensities, ch, weight), fn, mat, ls);
        }
        if (lightsonly) {
            problem.SetParameterBlockConstant(mat);
        } else {
            for (int i = 0; i < nummats; i++) {
                problem.SetParameterLowerBound(mat, i, 0);
                problem.SetParameterUpperBound(mat, i, 1);
            }
        }
        int curridx = 0;
        for (int i = 0; i < lightintensities.size(); i++) {
            RGBLight* rgbl = static_cast<RGBLight*>(lightintensities[i]);
            Light* l = rgbl->getLight(ch);
            if (lightintensities[i]->typeId() & LIGHTTYPE_ENVMAP) {
                vector<Vector3f> directions;
                vector<Vector3f> colors;
                vector<double> weights;
                for (int j = 0; j < mesh->NVertices(); j++) {
                    int lightid = LIGHTID(mesh->getLabel(j, MeshManager::LABEL_CHANNEL));
                    if (lightid == i+1) {
                        for (int k = 0; k < mesh->getVertexSampleCount(j); k++) {
                            Sample s = mesh->getSample(j, k);
                            colors.push_back(Vector3f(s.r, s.g, s.b));
                            directions.push_back(Vector3f(s.x, s.y, s.z));
                            weights.push_back(s.dA*s.confidence);
                        }
                    }
                }
                addObservedConstraints(l, &problem, ls, numlights, obslambda, directions, colors, weights, ch, curridx);
            }
            addCeres(l, &problem, ls, numlights, curridx);
            curridx += l->numParameters();
        }
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 100;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << "\n";
        for (int i = 0; i < nummats; i++) {
            materials[i](ch) = mat[i];
        }
        for (int i = 0; i < lightintensities.size(); i++) {
            RGBLight* rgbl = static_cast<RGBLight*>(lightintensities[i]);
            Light* l = rgbl->getLight(ch);
            for (int j = 0; j < l->numParameters(); j++) {
                l->coef(j) = ls[j];
            }
        }
    }
}
