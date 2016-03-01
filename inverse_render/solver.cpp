#include "solver.h"
#include <set>
#include <random>
#include <fstream>

using namespace std;

double error(vector<SampleData>& data, vector<int>& indices, vector<Material>& lights, double r, int ch) {
    double err = 0;
    for (int i = 0; i < indices.size(); ++i) {
        double d = data[indices[i]].radiosity(ch) - r*data[indices[i]].netIncoming(ch);
        for (int j = 0; j < data[indices[i]].lightamount.size(); ++j) {
            d -= r*data[indices[i]].lightamount[j]*lights[j](ch);
        }
        d *= 1-data[indices[i]].fractionUnknown;
        err += d*d;
    }
    return err;
}

void InverseRender::computeSamples(
        vector<SampleData>& data,
        vector<int> indices,
        int numsamples,
        double discardthreshold,
        bool saveImages,
        boost::function<void(int)> callback)
{
    hr->computeSamples(data, indices, numsamples, discardthreshold, saveImages?&images:NULL, callback);
    for (int i = 0; i < data.size(); ++i) {
        data[i].lightamount.resize(lights.size(), 0);
    }
}

int InverseRender::setupLightParameters(MeshManager* m) {
    lights.clear();
    coeftype.clear();

    int ret = 0;
    set<int> lightids;
    for (int i = 0; i < m->size(); i++) {
        char l = m->getLabel(i,0);
        if (l) lightids.insert(l);
    }
    for (auto lightinfo : lightids) {
        int n = LightTypeNumCoefficients[LIGHTTYPE(lightinfo)];
        for (int i = 0; i < n; i++) {
            coeftype.push_back(LIGHTTYPE(lightinfo));
        }
        ret += n;
    }
    lights.resize(ret);
    return ret;
}
void InverseRender::readVariablesMatlab(vector<SampleData>& data, string filename) {
    ifstream in(filename);
    string s;
    getline(in, s); // A = [
    while (getline(in, s)) {
        for (int i = 0; i < s.size(); i++) if (s[i] == ',') s[i] = ' ';
        data.push_back(SampleData());
        stringstream sin(s);
        while (sin.peek() != ';' && sin.peek() != ']') {
            float f;
            sin >> f;
            data.back().lightamount.push_back(f);
        }
        if (s[s.length()-2] == ']') break;
    }
    getline(in, s); // weights = [
    getline(in, s);
    for (int i = 0; i < s.size(); i++) if (s[i] == ';') s[i] = ' ';
    stringstream sin(s);
    for (int i = 0; i < data.size(); i++) {
        sin >> data[i].fractionUnknown;
    }
    getline(in, s); // direct = [
    getline(in, s);
    for (int i = 0; i < s.size(); i++) if (s[i] == ';') s[i] = ' ';
    sin.clear();
    sin.str(s);
    for (int i = 0; i < data.size(); i++) {
        sin >> data[i].fractionDirect;
    }
    for (int ch = 0; ch < 3; ++ch) {
        getline(in, s); // % Channel ch
        getline(in, s); // Bch = [
        getline(in, s);
        for (int i = 0; i < s.size(); i++) if (s[i] == ';') s[i] = ' ';
        sin.clear();
        sin.str(s);
        for (int i = 0; i < data.size(); ++i) {
            sin >> data[i].radiosity(ch);
        }
        getline(in, s); // Cch = [
        getline(in, s);
        for (int i = 0; i < s.size(); i++) if (s[i] == ';') s[i] = ' ';
        sin.clear();
        sin.str(s);
        for (int i = 0; i < data.size(); ++i) {
            sin >> data[i].netIncoming(ch);
        }
    }
}

void InverseRender::writeVariablesMatlab(vector<SampleData>& data, string filename) {
    ofstream out(filename);
    out << "A = [" << endl;
    for (int i = 0; i < data.size(); ++i) {
        for (int j = 0; j < lights.size(); ++j) {
            if (j < data[i].lightamount.size()) out << data[i].lightamount[j];
            else out << 0;
            if (j != lights.size()-1) out << ",";
        }
        if (i != data.size()-1) out << ";" << endl;
    }
    out << "];" << endl;
    out << "weights = [" << endl;
    for (int i = 0; i < data.size(); ++i) {
        out << data[i].fractionUnknown;
        if (i != data.size()-1) out << ";";
    }
    out << "];" << endl;
    out << "direct = [" << endl;
    for (int i = 0; i < data.size(); ++i) {
        out << data[i].fractionDirect;
        if (i != data.size()-1) out << ";";
    }
    out << "];" << endl;
    for (int ch = 0; ch < 3; ++ch) {
        out << "% Channel " << ch << endl;
        out << "B" << ch << " = [" << endl;
        for (int i = 0; i < data.size(); ++i) {
            out << data[i].radiosity(ch);
            if (i != data.size()-1) out << ";";
        }
        out << "];" << endl;
        out << "C" << ch << " = [" << endl;
        for (int i = 0; i < data.size(); ++i) {
            out << data[i].netIncoming(ch);
            if (i != data.size()-1) out << ";";
        }
        out << "];" << endl;
    }
    if (lights.size()) {
        for (int ch = 0; ch < 3; ch++) {
            out << "L" << ch << " = [" << endl;
            for (int i = 0; i < lights.size(); i++) {
                out << lights[i](ch);
                if (i != lights.size()-1) out << ";";
            }
            out << "];" << endl;
        }
    }
}
void InverseRender::writeVariablesBinary(vector<SampleData>& data, string filename) {
    ofstream out(filename, ofstream::binary);
    uint32_t sz = data.size();
    out.write((char*) &sz, 4);
    sz = lights.size();
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
        for (int j = 0; j < lights.size(); ++j) {
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

void InverseRender::writeLightsToTextFile(string filename) {
    ofstream out(filename);
    out << lights.size() << endl;
    for (int i = 0; i < lights.size(); i++) {
        for (int ch = 0; ch < 3; ch++) {
            out << lights[i](ch) << " ";
        }
        out << endl;
    }
}
