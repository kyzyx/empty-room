#include "rendering/sh.h"
#include "rendering/envmap.h"
#include "datamanager/imageio.h"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;

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
    bool cubemap = false;
    bool negative = false;
    bool sh = true;
    for (int i = 4; i < argc; i++) {
        if (strcmp(argv[i], "--cubemap") == 0) {
            cubemap = true;
            sh = false;
        }
        else if (strcmp(argv[i], "--allow-negative-values") == 0) {
            negative = true;
        }
    }

    vector<double> coef[3];
    int lines;
    in >> lines;
    string tmp;
    getline(in, tmp);
    for (int z = 0; z < lines; z++) {
        for (int i = 0; i < 3; i++) {
            double d;
            in >> d;
            coef[i].push_back(d);
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
                if (cubemap) v = sampleEnvmap(theta, -phi, coef[ch], envmapres);
                else if (sh) v = sampleSH(theta, -phi, coef[ch]);
                if (!negative) v = max(v, 0.);
                image[3*(i*res*2+j) + ch] = v;
            }
        }
    }
    ImageIO::writeExrImage(outfile, image, res*2, res, 3);
}
