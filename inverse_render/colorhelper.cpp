#include "colorhelper.h"

#include <iostream>
#include <fstream>
#include <cstdio>

#include "rgbe.h"
#include <png.h>

//#define OUTPUT_RADIANCE_CAMERAS

using namespace std;

bool ColorHelper::load(string cameraFile) {
    readCameraFile(cameraFile);
    for (int i = 0; i < filenames.size(); ++i) {
        if (!readImage(filenames[i])) {
            cerr << "Error reading image " << filenames[i] << endl;
            return false;
        }
    }
}
bool endswith(const string& s, string e) {
    if (s.length() > e.length())
        return s.compare(s.length()-e.length(), e.length(), e) == 0;
    else
        return false;
}
bool ColorHelper::readImage(const string& filename) {
    if (endswith(filename, ".png"))
        return readPngImage(filename);
    else if (endswith(filename, ".hdr") || endswith(filename, ".pic"))
        return readHdrImage(filename);
    else
        return false;
}
bool ColorHelper::readHdrImage(const string& filename) {
    int width, height;
    rgbe_header_info info;
    FILE* file = fopen(filename.c_str(), "rb");

    RGBE_ReadHeader(file, &width, &height, &info);
    float* image = new float[3*width*height];
    RGBE_ReadPixels_RLE(file, image, width, height);
    float expadj = info.exposure;
    for (int i = 0; i < width*height*3; ++i) {
        image[i] /= expadj;
    }
    // Flip y-coordinate (standard is 0,0 at top)
    float* row = new float[3*width];
    for (int i = 0; i < height/2; ++i) {
        memcpy(row, image+3*(i*width), 3*width*sizeof(float));
        memcpy(image+3*(i*width), image+3*((height-i-1)*width), 3*width*sizeof(float));
        memcpy(image+3*((height-i-1)*width), row, 3*width*sizeof(float));
    }
    delete [] row;
    data.push_back((char*)image);
    fclose(file);
    return true;
}
// From http://blog.nobel-joergensen.com/2010/11/07/loading-a-png-as-texture-in-opengl-using-libpng/
bool ColorHelper::readPngImage(const string& filename)
{
    png_structp png_ptr;
    png_infop info_ptr;
    unsigned int sig_read = 0;
    int color_type, interlace_type;
    FILE *fp;

    if ((fp = fopen(filename.c_str(), "rb")) == NULL)
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

bool ColorHelper::readCameraFile(const string& filename)
{
    // Format:
    //    FrameCount Width Height hfov
    //    filename X Y Z [Up] [Towards]
    //    Angles in degrees
    try {
        ifstream in(filename.c_str());
        int frames, w, h;
        double hfov;
        in >> frames >> w >> h >> hfov;
        double foc = w/(2*tan(hfov*M_PI/180));
        double a,b,c;
        string s;
        for (int i = 0; i < frames; ++i) {
            CameraParams* curr = new CameraParams;
            in >> s >> a >> b >> c;
            filenames.push_back(s);
#ifdef OUTPUT_RADIANCE_CAMERAS
            printf("view= pos%d rvu -vtv -vp %f %f %f", i, a, b, c);
#endif
            curr->pos.Reset(a,b,c);
            in >> a >> b >> c;
            curr->up.Reset(a,b,c);
            in >> a >> b >> c;
            curr->towards.Reset(a,b,c);
#ifdef OUTPUT_RADIANCE_CAMERAS
            printf(" -vu %f %f %f", curr->up[0], curr->up[1], curr->up[2]);
            printf(" -vd %f %f %f", curr->towards[0], curr->towards[1], curr->towards[2]);
            double vfov = hfov*curr->height/curr->width;
            printf(" -vh %f -vv %f -vo 0 -va 0 -vs 0 -vl 0\n", hfov, vfov);
#endif
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
