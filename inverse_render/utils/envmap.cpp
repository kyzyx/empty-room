#include "rendering/sh.h"
#include "datamanager/imageio.h"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;

double sample(double theta, double phi, vector<double>& coef) {
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

int main(int argc, char** argv) {
    if (argc < 4) {
        cout << "Usage: envmap coefficents.txt output.exr resolution [-cubemap]" << endl;
        cout << "Generates environment maps from spherical harmonic coefficients." << endl;
        cout << "By default, generates one [2*resolution]x[resolution] image such" << endl;
        cout << "that u = phi, v = theta. This is suitable for use in PBRT." << endl;
        cout << endl;
        cout << "Alternatively, specifying the -cubemap option will generate" << endl;
        cout << "a cubemap with 6 faces of [resolution]x[resolution]" << endl;
        return 0;
    }

    ifstream in(argv[1]);
    string outfile = argv[2];
    int res = atoi(argv[3]);
    bool cubemap = false;
    if (argc > 4) {
        if (strcmp(argv[4], "-cubemap") == 0) {
            cubemap = true;
        }
    }

    vector<double> coef[3];
    while (!in.eof()) {
        for (int i = 0; i < 3; i++) {
            double d;
            in >> d;
            coef[i].push_back(d);
        }
    }
    int numbands = sqrt(coef[0].size());

    float* image = new float[res*res*2*3];
    for (int i = 0; i < res; i++) {
        for (int j = 0; j < res*2; j++) {
            for (int ch = 0; ch < 3; ch++) {
                double theta = M_PI*(j-res)/res;
                double phi = M_PI*i/res;
                image[3*(i*res*2+j) + ch] = sample(theta, phi, coef[ch]);
            }
        }
    }
    ImageIO::writeExrImage(outfile, image, res*2, res, 3);
}
