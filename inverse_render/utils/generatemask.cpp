#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "datamanager/imageio.h"

using namespace std;
using namespace cv;

int minpix = 20;
int maxpix = 230;

float ConfidenceFn(unsigned char f) {
    if (f < minpix || f > maxpix) return 0;
    return 1;
    //return f > 127? (255-f)/127. : f/127.; // Hat function
}
float confidence(unsigned char* rgb) {
    return min(min(ConfidenceFn(rgb[0]),
                   ConfidenceFn(rgb[1])),
                   ConfidenceFn(rgb[2]));
}
void generateEdgeMask(Mat& image, Mat& mask) {
    const double thresh = 30;
    const int s = 2;
    Mat edge_x, edge_y, grad;
    Sobel(image, edge_x, CV_32F, 1, 0);
    Sobel(image, edge_y, CV_32F, 0, 1);
    pow(edge_x, 2, edge_x);
    pow(edge_y, 2, edge_y);
    add(edge_x, edge_y, grad);
    sqrt(grad, grad);
    vector<Mat> channels;
    split(grad, channels);
    max(channels[0], channels[1], grad);
    max(grad, channels[2], grad);
    threshold(grad, grad, thresh, 1, THRESH_BINARY_INV);
    Mat C[3] = {grad, grad, grad};
    merge(C, 3, mask);
    mask.convertTo(mask, CV_8UC3);
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2*s+1, 2*s+1), Point(s,s));
    erode(mask, mask, element);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "Usage: generatemask camfile.cam [blur kernel size]" << endl;
        return 0;
    }
    ifstream in(argv[1]);
    int sz;
    int w, h;
    int blur = 0;
    if (argc > 2) {
        blur = atoi(argv[2]);
        if (blur > 1 && blur%2 == 0) blur++;
    }

    string line;
    getline(in, line);
    stringstream infoin(line);
    infoin >> sz >> w >> h;

    Mat im, mask, masked;
    unsigned char* image = new unsigned char[w*h*3];
    float* conf = new float[w*h];
    im = Mat(h, w, CV_8UC3, image);
    for (int i = 0; i < sz; i++) {
        getline(in, line);
        string filename = line.substr(0, line.find(' '));
        map<string, float> header;
        int ww, hh;
        ImageIO::readRGBImage(filename, image, ww, hh);
        if (blur) GaussianBlur(im, im, Size(blur,blur), blur/3);
        generateEdgeMask(im, mask);
        multiply(mask, im, masked);
        for (int j = 0; j < w*h; j++) {
            conf[j] = confidence(masked.ptr() + 3*j);
        }
        string conffilename = ImageIO::replaceExtension(filename, "conf");
        ImageIO::writeScalarMap(conffilename, conf, w, h, sizeof(float));
        cout << filename << endl;
    }
}
