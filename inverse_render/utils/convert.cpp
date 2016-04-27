#include "datamanager/imageio.h"
#include <string>
#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/console/parse.h>

using namespace std;

float image[3*2560*1440];
float im2[3*2560*1440];
inline unsigned char fclamp(float f) {
    if (f > 255) return 255;
    if (f < 0) return 0;
    return f;
}
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Usage: convertimage inputimage outputimage [-intensityscale outputscale] [-colorscale scaler,scaleg,scaleb] [-scale scale] [-flip_y] [-flip_x] [-or inputimage2]" << endl;
        return 0;
    }
    double intensityscale[3];
    float scale = 1;
    float gamma = 0;
    bool linear = false;
    string orfile = "";
    bool do_or = false;
    for (int i = 0; i < 3; i++) intensityscale[i] = 1;
    if (pcl::console::find_argument(argc, argv, "-intensityscale") >= 0) {
        double d;
        pcl::console::parse_argument(argc, argv, "-intensityscale", d);
        for (int i = 0; i < 3; i++) intensityscale[i] = d;
    }
    if (pcl::console::find_argument(argc, argv, "-colorscale") >= 0) {
        string colorscale;
        pcl::console::parse_argument(argc, argv, "-colorscale", colorscale);
        vector<string> s;
        split(colorscale, ',', s);
        for (int i = 0; i < 3; i++) intensityscale[i] = atof(s[i].c_str());
    }
    if (pcl::console::find_argument(argc, argv, "-scale") >= 0) {
        pcl::console::parse_argument(argc, argv, "-scale", scale);
    }
    if (pcl::console::find_argument(argc, argv, "-gamma") >= 0) {
        pcl::console::parse_argument(argc, argv, "-gamma", gamma);
    }
    if (pcl::console::find_switch(argc, argv, "-linear")) {
        linear = true;
    }
    if (pcl::console::find_argument(argc, argv, "-or") >= 0) {
        pcl::console::parse_argument(argc, argv, "-or", orfile);
        do_or = true;
    }
    bool flip_y = false;
    bool flip_x = false;
    if (pcl::console::find_switch(argc, argv, "-flip_y")) {
        flip_y = true;
    }
    if (pcl::console::find_switch(argc, argv, "-flip_x")) {
        flip_x = true;
    }
    int w, h;
    if (!ImageIO::readFloatImage(argv[1], image, w, h)) {
        cout << "Error reading file" << endl;
        return 0;
    }
    if (do_or && !ImageIO::readFloatImage(orfile.c_str(), im2, w, h)) {
        cout << "Error reading file" << endl;
        return 0;
    }
    if (flip_x || flip_y) ImageIO::flip(image, w, h, sizeof(float)*3, flip_x, flip_y);
    //string fmt = argv[2];
    //string outfile = ImageIO::replaceExtension(argv[1], fmt);
    string outfile = argv[2];
    string fmt = outfile.substr(outfile.find_last_of(".")+1);
    for (int i = 0; i < w*h; i++) {
        bool iszero = true;
        for (int j = 0; j < 3; j++) {
            if (gamma > 0) image[3*i+j] = pow(image[3*i+j], gamma);
            image[3*i+j] *= intensityscale[j];
            if (!linear && gamma > 0) image[3*i+j] = pow(image[3*i+j], 1./gamma);
            if (image[3*i+j]) iszero = false;
        }
        if (do_or && iszero) {
            for (int j = 0; j < 3; j++) {
                image[3*i+j] = im2[3*i+j];
            }
        }
    }
    cv::Mat im(h, w, CV_32FC3, image);
    cv::Mat res;
    if (scale != 1) {
        if (scale > 1) {
            cv::resize(im, res, cv::Size(), scale, scale, cv::INTER_LINEAR);
        } else {
            cv::resize(im, res, cv::Size(), scale, scale, cv::INTER_AREA);
        }
    } else {
        res = im;
    }
    if (fmt == "png") {
        unsigned char* img = new unsigned char[res.cols*res.rows*3];
        float* p = (float*) res.data;
        for (int i = 0; i < res.cols*res.rows*3; i++) {
            img[i] = fclamp(255*p[i]);
        }
        if (!ImageIO::writePngImage(outfile, img, res.cols, res.rows)) {
            cout << "Error writing file" << endl;
        }
        delete img;
    }
    else if (fmt == "exr") {
        ImageIO::writeExrImage(outfile, (float*) res.data, res.cols, res.rows);
    }
    else {
        cout << "Unknown format " << fmt << endl;
    }
}
