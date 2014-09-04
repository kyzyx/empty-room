#include "colorhelper.h"

#include <iostream>
#include <fstream>
#include <cstdio>

#include <png.h>

using namespace std;

bool ColorHelper::load(std::string imageListFile, std::string cameraFile) {
    if (!readImageNames(imageListFile)) return false;
    for (int i = 0; i < filenames.size(); ++i) {
        if (!readImage(i)) {
            cerr << "Error reading image " << filenames[i] << endl;
            return false;
        }
    }
    return readMayaCameraFile(cameraFile);
}
// From http://blog.nobel-joergensen.com/2010/11/07/loading-a-png-as-texture-in-opengl-using-libpng/
bool ColorHelper::readImage(int n)
{
    png_structp png_ptr;
    png_infop info_ptr;
    unsigned int sig_read = 0;
    int color_type, interlace_type;
    FILE *fp;

    if ((fp = fopen(filenames[n].c_str(), "rb")) == NULL)
        return false;

    png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (png_ptr == NULL) {
        fclose(fp);
        return false;
    }

    info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == NULL) {
        fclose(fp);
        png_destroy_read_struct(&png_ptr, NULL, NULL);
        return false;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(fp);
        return false;
    }
    png_init_io(png_ptr, fp);
    png_set_sig_bytes(png_ptr, sig_read);
    png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_STRIP_ALPHA | PNG_TRANSFORM_PACKING | PNG_TRANSFORM_EXPAND, NULL);
    png_uint_32 width, height;
    int bit_depth;
    png_get_IHDR(png_ptr, info_ptr, &width, &height, &bit_depth, &color_type,
                 &interlace_type, NULL, NULL);

    unsigned int row_bytes = png_get_rowbytes(png_ptr, info_ptr);
    char* imagedata = new char[width*height*3];

    png_bytepp row_pointers = png_get_rows(png_ptr, info_ptr);

    for (int i = 0; i < height; i++) {
        memcpy(imagedata+(row_bytes * (height-1-i)), row_pointers[i], row_bytes);
    }

    data.push_back(imagedata);

    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
    fclose(fp);
    return true;
}

bool ColorHelper::readImageNames(string filename)
{
    try {
        ifstream in(filename.c_str());
        string s;
        getline(in, s);
        while (!in.eof()) {
            filenames.push_back(s);
            getline(in, s);
        }
    } catch(...) {
        return false;
    }
    return true;
}

bool ColorHelper::readMayaCameraFile(string filename)
{
    // Format:
    //    FrameCount Width Height hfov
    //    X Y Z [Up] [Towards]
    //    Angles in degrees
    try {
        ifstream in(filename.c_str());
        int frames, w, h;
        double hfov;
        in >> frames >> w >> h >> hfov;
        double foc = w/(2*tan(hfov*M_PI/180));
        double a,b,c;
        for (int i = 0; i < frames; ++i) {
            CameraParams* curr = new CameraParams;
            in >> a >> b >> c;
            curr->pos.Reset(a,b,c);
            in >> a >> b >> c;
            curr->up.Reset(a,b,c);
            in >> a >> b >> c;
            curr->towards.Reset(a,b,c);
            curr->right = curr->towards%curr->up;
            curr->width = w;
            curr->height = h;
            curr->focal_length = foc;
            cameras.push_back(curr);
        }
    } catch (...) {
        return false;
    }
    return true;
}
