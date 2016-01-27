#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "datamanager/imageio.h"

using namespace std;

int minpix = 20;
int maxpix = 230;

float ConfidenceFn(unsigned char f) {
    return f > 127? (256-f)/127. : f/127.; // Hat function, but with 255 > 0
}
float confidence(unsigned char* rgb) {
    return max(max(ConfidenceFn(rgb[0]),
                   ConfidenceFn(rgb[1])),
                   ConfidenceFn(rgb[2]));
}

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "Usage: generateconfidence camfile.cam" << endl;
        return 0;
    }
    ifstream in(argv[1]);
    int sz;
    int w, h;

    string line;
    ImageIO::readCameraFileHeader(in, sz, w, h);

    unsigned char* image = new unsigned char[w*h*3];
    float* conf = new float[w*h];
    for (int i = 0; i < sz; i++) {
        getline(in, line);
        string filename = line.substr(0, line.find(' '));
        int ww, hh;
        ImageIO::readRGBImage(filename, image, ww, hh);
        for (int j = 0; j < w*h; j++) {
            conf[j] = confidence(image + 3*j);
        }
        string conffilename = ImageIO::replaceExtension(filename, "conf");
        ImageIO::writeScalarMap(conffilename, conf, w, h, sizeof(float));
        cout << filename << endl;
    }
}
