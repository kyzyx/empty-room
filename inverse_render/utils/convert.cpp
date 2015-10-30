#include "datamanager/imageio.h"
#include <string>
#include <iostream>
#include <cstdlib>

using namespace std;

float image[3*2560*1440];
inline unsigned char fclamp(float f) {
    if (f > 255) return 255;
    if (f < 0) return 0;
    return f;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Usage: convertimage inputimage outputformat [outputscale | scaler scaleg scaleb]" << endl;
        return 0;
    }
    int w, h;
    if (!ImageIO::readFloatImage(argv[1], image, w, h)) {
        cout << "Error reading file" << endl;
        return 0;
    }
    //string fmt = argv[2];
    //string outfile = ImageIO::replaceExtension(argv[1], fmt);
    string outfile = argv[2];
    string fmt = outfile.substr(outfile.find_last_of(".")+1);
    if (argc > 3) {
        if (argc == 6) {
            double f[3];
            for (int ch = 0; ch < 3; ch++) f[ch] = atof(argv[3+ch]);
            for (int i = 0; i < w*h; i++) {
                for (int j = 0; j < 3; j++) {
                    image[3*i+j] *= f[j];
                }
            }
        } else {
            double f = atof(argv[3]);
            for (int i = 0; i < w*h*3; i++) {
                image[i] *= f;
            }
        }
    }
    if (fmt == "png") {
        unsigned char* im = new unsigned char[w*h*3];
        for (int i = 0; i < w*h*3; i++) {
            im[i] = fclamp(255*image[i]);
        }
        if (!ImageIO::writePngImage(outfile, im, w, h)) {
            cout << "Error writing file" << endl;
        }
        delete im;
    }
    else if (fmt == "exr") {
        ImageIO::writeExrImage(outfile, image, w, h);
    }
    else {
        cout << "Unknown format " << fmt << endl;
    }
}
