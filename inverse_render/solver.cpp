#include "solver.h"
#include "wall_finder.h"
#include <random>
#include <Eigen/Dense>
#include <fstream>

using namespace std;
using namespace Eigen;


void InverseRender::solve(vector<SampleData>& data) {
    if (calculateWallMaterialFromUnlit(data)) {
        cout << "Material estimate: (" << wallMaterial(0) << "," << wallMaterial(1) << "," << wallMaterial(2) << ")" << endl;
    }
    if (!numlights) return;
    lights.resize(numlights);
    solveLights(data);
    for (int i = 0; i < lights.size(); ++i) {
        cout << "Light estimate " << i << ": (" << lights[i](0)/M_PI << "," << lights[i](1)/M_PI << "," << lights[i](2)/M_PI << ")" << endl;
    }
}
void InverseRender::computeSamples(
        vector<SampleData>& data,
        vector<int> indices,
        int numsamples,
        double discardthreshold,
        bool saveImages)
{
    if (saveImages) {
        images = new float*[numsamples*2];
    }
    hr.computeSamples(data, indices, numsamples, discardthreshold, images);
}

void InverseRender::writeVariablesMatlab(vector<SampleData>& data, string filename) {
    ofstream out(filename);
    out << "A = [";
    for (int i = 0; i < data.size(); ++i) {
        for (int j = 0; j < numlights; ++j) {
            if (j < data[i].lightamount.size()) out << data[i].lightamount[j];
            else out << 0;
            if (j != numlights-1) out << ",";
        }
        if (i != data.size()-1) out << ";" << endl;
    }
    out << "];" << endl;
    out << "weights = [";
    for (int i = 0; i < data.size(); ++i) {
        out << data[i].fractionUnknown;
        if (i != data.size()-1) out << ";";
    }
    out << "];" << endl;
    for (int ch = 0; ch < 3; ++ch) {
        out << "% Channel " << ch << endl;
        out << "B" << ch << " = [";
        for (int i = 0; i < data.size(); ++i) {
            out << data[i].radiosity(ch);
            if (i != data.size()-1) out << ";";
        }
        out << "];" << endl;
        out << "C" << ch << " = [";
        for (int i = 0; i < data.size(); ++i) {
            out << data[i].netIncoming(ch);
            if (i != data.size()-1) out << ";";
        }
        out << "];" << endl;
    }
}
void InverseRender::writeVariablesBinary(vector<SampleData>& data, string filename) {
    ofstream out(filename, ofstream::binary);
    uint32_t sz = data.size();
    out.write((char*) &sz, 4);
    sz = numlights;
    out.write((char*) &sz, 4);
    for (int i = 0; i < data.size(); ++i) {
        out.write((char*)&(data[i].fractionUnknown), sizeof(float));
        out.write((char*)&(data[i].vertexid), sizeof(int));
        out.write((char*)&(data[i].radiosity(0)), sizeof(float));
        out.write((char*)&(data[i].radiosity(1)), sizeof(float));
        out.write((char*)&(data[i].radiosity(2)), sizeof(float));
        out.write((char*)&(data[i].netIncoming(0)), sizeof(float));
        out.write((char*)&(data[i].netIncoming(1)), sizeof(float));
        out.write((char*)&(data[i].netIncoming(2)), sizeof(float));
        for (int j = 0; j < numlights; ++j) {
            static float zero = 0.f;
            if (j < data[i].lightamount.size()) out.write((char*)&(data[i].lightamount[j]), sizeof(float));
            else out.write((char*)&zero, sizeof(float));
        }
    }
}

void InverseRender::loadVariablesBinary(vector<SampleData>& data, string filename) {
    ifstream in(filename, ifstream::binary);
    uint32_t sz;
    in.read((char*) &sz, 4);
    data.resize(sz);
    in.read((char*) &sz, 4);
    for (int i = 0; i < data.size(); ++i) {
        data[i].lightamount.resize(sz);
        in.read((char*)&(data[i].fractionUnknown), sizeof(float));
        in.read((char*)&(data[i].vertexid), sizeof(int));
        in.read((char*)&(data[i].radiosity(0)), sizeof(float));
        in.read((char*)&(data[i].radiosity(1)), sizeof(float));
        in.read((char*)&(data[i].radiosity(2)), sizeof(float));
        in.read((char*)&(data[i].netIncoming(0)), sizeof(float));
        in.read((char*)&(data[i].netIncoming(1)), sizeof(float));
        in.read((char*)&(data[i].netIncoming(2)), sizeof(float));
        for (int j = 0; j < data[i].lightamount.size(); ++j) {
            in.read((char*)&(data[i].lightamount[j]), sizeof(float));
        }
    }
}

bool InverseRender::calculateWallMaterialFromUnlit(vector<SampleData>& data) {
    vector<double> estimates[3];
    vector<double> weights;
    for (int i = 0; i < data.size(); ++i) {
        bool unlit = true;
        for (int j = 0; j < data[i].lightamount.size(); ++j) {
            if (data[i].lightamount[j] > 0) {
                unlit = false;
                break;
            }
        }
        if (unlit) {
            double w = 1 - data[i].fractionUnknown;
            bool valid = true;
            for (int ch = 0; ch < 3; ++ch) {
                if (data[i].radiosity(ch)*w/data[i].netIncoming(ch) > 1) {
                    valid = false;
                    break;
                }
            }
            if (valid) {
                for (int ch = 0; ch < 3; ++ch) {
                    estimates[ch].push_back(data[i].radiosity(ch)*w/data[i].netIncoming(ch));
                }
                weights.push_back(w);
            }
        }
    }
    if (estimates[0].size() == 0) return false;
    cout << "Unlit estimates: " << estimates[0].size() << endl;

    double totweight = accumulate(weights.begin(), weights.end(), 0.);
    double mean[3];
    double stddev[3];
    double newmean[3];
    double count = 0;
    for (int ch = 0; ch < 3; ++ch) {
        mean[ch] = 0;
        for (int i = 0; i < estimates[ch].size(); ++i) {
            mean[ch] += estimates[ch][i]*weights[i];
        }
        mean[ch] /= totweight;
        stddev[ch] = 0;
    }
    for (int i = 0; i < estimates[0].size(); ++i) {
        for (int ch = 0; ch < 3; ++ch) {
            cout << estimates[ch][i] << " ";
            stddev[ch] += weights[i]*(estimates[ch][i]-mean[ch])*(estimates[ch][i]-mean[ch]);
        }
        cout << endl;
    }
    for (int ch = 0; ch < 3; ++ch) {
        stddev[ch] = sqrt(stddev[ch]/totweight);
    }
    for (int mult = 1; count == 0; ++mult) {
        for (int ch = 0; ch < 3; ++ch) {
            newmean[ch] = 0;
        }
        for (int i = 0; i < estimates[0].size(); ++i) {
            int bound = 0;
            for (int ch = 0; ch < 3; ++ch) {
                if (estimates[ch][i] < mean[ch] - mult*stddev[ch]) bound = -1;
                else if (estimates[ch][i] > mean[ch] + mult*stddev[ch]) bound = 1;
            }

            if (bound < 0) continue;
            if (bound > 0) break;
            count += weights[i];
            for (int ch = 0; ch < 3; ++ch) {
                newmean[ch] += weights[i]*estimates[ch][i];
            }
        }
    }
    for (int ch = 0; ch < 3; ++ch) {
        wallMaterial(ch) = newmean[ch]/count;
    }
    return true;
}


const double threshold = 0.01; // % change above which we are not converged
bool InverseRender::solveLights(vector<SampleData>& data) {
    // Weighted linear least squares for each channel
    double maxchange = 0.;
    for (int ch = 0; ch < 3; ++ch) {
        VectorXd b(data.size());
        MatrixXd A(data.size(), numlights);
        for (int i = 0; i < data.size(); ++i) {
            b[i] = (1-data[i].fractionUnknown)*data[i].radiosity(ch) - wallMaterial(ch)*data[i].netIncoming(ch);
            for (int j = 0; j < data[i].lightamount.size(); ++j) {
                A(i,j) = (1-data[i].fractionUnknown)*wallMaterial(ch)*data[i].lightamount[j];
            }
        }
        VectorXd x = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
        for (int i = 0; i < numlights; ++i) {
            if (lights[i](ch) > 0) {
                double delta = abs(lights[i](ch) - x[i]);
                double percent = delta/lights[i](ch);
                if (percent > maxchange) maxchange = percent;
            }
            lights[i](ch) = x[i];
        }
    }
    return maxchange < threshold;
}

bool InverseRender::solveMaterials(vector<SampleData>& data) {
    // Weighted average over sample points
    bool converged = true;
    for (int ch = 0; ch < 3; ++ch) {
        double prev = wallMaterial(ch);
        wallMaterial(ch) = 0;
        double tot = 0;
        for (int i = 0; i < data.size(); ++i) {
            double totalin = data[i].netIncoming(ch);
            for (int j = 0; j < data[i].lightamount.size(); ++j) {
                totalin += lights[j](ch)*data[i].lightamount[j]*(1-data[i].fractionUnknown);
            }
            double p = data[i].radiosity(ch)*(1-data[i].fractionUnknown)/totalin;
            wallMaterial(ch) += p;
            tot += 1-data[i].fractionUnknown;
        }
        wallMaterial(ch) /= tot;
        if (abs(prev - wallMaterial(ch)) > threshold) converged = false;
    }
    return converged;
}
