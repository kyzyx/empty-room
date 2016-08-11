#include "lighting/light.h"
#include "lighting/sh.h"
#include "lighting/cubemap.h"
#include "datamanager/imageio.h"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;

double sampleLine(double y, double x, vector<double>& coef) {
    double theta = atan2(y, x);
    if (theta < 0) theta += (2*M_PI);
    int idx = coef.size()*theta/(2*M_PI);
    return coef[idx];
}
double sampleSH(double theta, double phi, vector<double>& coef) {
    double ret = 0;
    int l = 0;
    int m = 0;
    for (int i = 0; i < coef.size(); i++) {
        if (m > l) {
            l++;
            m = -l;
        }
        ret += coef[i]*SH(l, m, theta, phi);
        m++;
    }
    return ret;
}
double sampleEnvmap(double theta, double phi, vector<double>& coef, int res) {
    double s = sin(theta);
    int n = getEnvmapCell(s*cos(phi), s*sin(phi), cos(theta), res);
    return coef[n];
}

int main(int argc, char** argv) {
    if (argc < 4) {
        cout << "Usage: envmap coefficents.txt output.exr resolution [--cubemap] [--allow-negative-values]" << endl;
        cout << "Generates a [2*resolution]x[resolution] environment map such" << endl;
        cout << "that u = phi, v = theta. This is suitable for use in PBRT." << endl;
        cout << endl;
        cout << "By default, uses spherical harmonic coefficients." << endl;
        cout << "Alternatively, specifying the -cubemap option will use" << endl;
        cout << "a constant basis environment map." << endl;
        return 0;
    }

    ifstream in(argv[1]);
    string outfile = argv[2];
    int res = atoi(argv[3]);
    int lighttype = LIGHTTYPE_SH;
    bool negative = false;
    bool binary = false;
    bool newformat = false;
    for (int i = 4; i < argc; i++) {
        if (strcmp(argv[i], "--cubemap") == 0) {
            lighttype = LIGHTTYPE_ENVMAP;
        } else if (strcmp(argv[i], "--linelight") == 0) {
            lighttype = LIGHTTYPE_LINE;
        } else if (strcmp(argv[i], "--allow-negative-values") == 0) {
            negative = true;
        } else if (strcmp(argv[i], "--newformat") == 0) {
            newformat = true;
        } else if (strcmp(argv[i], "--binary") == 0) {
            binary = true;
        }
    }

    vector<double> coef[3];
    if (newformat) {
        vector<vector<Light*> > lights;
        lights.resize(3);
        readLightsFromFile(argv[1], lights, binary);
        for (int z = 0; z < 3; z++) {
            for (int i = 0; i < lights[z][0]->numParameters(); i++) {
                coef[z].push_back(lights[z][0]->coef(i));
            }
        }
        if (lights[0][0]->typeId() & LIGHTTYPE_LINE) {
            lighttype = LIGHTTYPE_LINE;
        } else if (lights[0][0]->typeId() & LIGHTTYPE_SH) {
            lighttype = LIGHTTYPE_SH;
        } else if (lights[0][0]->typeId() & LIGHTTYPE_ENVMAP) {
            lighttype = LIGHTTYPE_ENVMAP;
        }
    } else {
        int lines;
        in >> lines;
        string tmp;
        getline(in, tmp);
        for (int i = 0; i < 3; i++) {
            for (int z = 0; z < lines; z++) {
                double d;
                in >> d;
                coef[i].push_back(d);
            }
        }
    }

    float* image = new float[res*res*2*3];
    int numbands = sqrt(coef[0].size());
    int envmapres = sqrt(coef[0].size()/6);

    for (int i = 0; i < res; i++) {
        for (int j = 0; j < res*2; j++) {
            for (int ch = 0; ch < 3; ch++) {
                double phi = M_PI*(j-res)/res;
                double theta = M_PI*i/res;
                double v;
                if (lighttype == LIGHTTYPE_ENVMAP) v = sampleEnvmap(theta, -phi, coef[ch], envmapres);
                else if (lighttype == LIGHTTYPE_SH) v = sampleSH(theta, -phi, coef[ch]);
                else if (lighttype == LIGHTTYPE_LINE) v = sampleLine((j-res), (res/2-i), coef[ch]);
                if (!negative) v = max(v, 0.);
                image[3*(i*res*2+j) + ch] = v;
            }
        }
    }
    ImageIO::writeExrImage(outfile, image, res*2, res, 3);
}
