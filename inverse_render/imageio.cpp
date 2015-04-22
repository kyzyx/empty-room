#include "imageio.h"
#include <OpenEXR/ImfRgbaFile.h>
#include <OpenEXR/ImfArray.h>
#include <OpenEXR/ImfHeader.h>

#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>

#include "rgbe.h"
#include <png.h>
#include <sys/stat.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace ImageIO {

using namespace std;
using namespace pcl;
using namespace Imf;
using namespace Imath;

bool endswith(const string& s, string e) {
    if (s.length() > e.length())
        return s.compare(s.length()-e.length(), e.length(), e) == 0;
    else
        return false;
}

bool readDepthMap(const string& filename, void* image, int w, int h) {
    int width = 0;
    int height = 0;
    bool success = readPcdDepthImage(filename, (float**) &image, width, height);
    return width == w && height == h && success;
}

bool readScalarMap(const string& filename, void* image, int w, int h, int bytes) {
    try {
        ifstream in(filename.c_str(), ifstream::binary);
        char* p = (char*) image;
        p += (h-1)*w*bytes;
        for (int i = 0; i < h; ++i) {
            in.read(p, w*bytes);
            p -= w*bytes;
        }
        return true;
    } catch(...) {
        return false;
    }
}

bool readFloatImage(const string& filename, void* image, int w, int h, int channels) {
    int width = 0;
    int height = 0;

    if (endswith(filename, ".hdr") || endswith(filename, ".pic")) {
        if (channels == 3) {
            if (!readHdrImage(filename, (float**) &image, width, height, true)) return false;
        } else {
            float* tmp;
            if (!readHdrImage(filename, &tmp, width, height)) return false;

            float* p = (float*) image;
            for (int i = 0; i < width*height; ++i) {
                for (int j = 0; j < channels; ++j) {
                    *(p++) = tmp[i*3+j];
                }
            }
            delete tmp;
        }
    } else if (endswith(filename, ".png")) {
        char* tmp;
        if (!readPngImage(filename, &tmp, width, height)) return false;
        // Upgrade to float 0-1
        float* p = (float*) image;
        for (int i = 0; i < width*height; ++i) {
            for (int j = 0; j < channels; ++j) {
                *(p++) = tmp[i*3+j]/255.;
            }
        }
        delete tmp;
    } else {
        // Default to EXR format
        if (!readExrImage(filename, (float**) &image, width, height, channels, true)) return false;
    }

    return width == w && height == h;
}

bool readRGBImage(const string& filename, void* image, int w, int h, int channels) {
    int width = 0;
    int height = 0;
    if (channels == 3) {
        if (!readPngImage(filename, (char**) &image, width, height, true)) return false;
    } else {
        char* tmp;
        if (!readPngImage(filename, &tmp, width, height)) return false;
        char* p = (char*) image;
        for (int i = 0; i < width*height; ++i) {
            for (int j = 0; j < channels; ++j) {
                *(p++) = tmp[i*3+j];
            }
        }
        delete tmp;
    }

    return width == w && height == h;
}

bool readPcdDepthImage(const string& filename,
        float** image,
        int& width,
        int& height,
        bool preallocated)
{
    try {
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
        if (io::loadPCDFile<PointXYZ>(filename, *cloud) == -1) {
            return false;
        }
        width = cloud->width;
        height = cloud->height;
        if (!preallocated) *image = new float[width*height];
        float* im = *image;
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                PointXYZ p = cloud->at(j,i);
                if (isnan(p.x) || isnan(p.y) || isnan(p.z)) {
                    im[(height-i-1)*width+j] = numeric_limits<float>::infinity();
                } else {
                    im[(height-i-1)*width+j] = sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
                }
            }
        }
    } catch(...) {
        return false;
    }
    return true;
}
bool readExrImage(const string& filename,
        float** image,
        int& width,
        int& height,
        int channels,
        bool preallocated)
{
    RgbaInputFile f(filename.c_str());
    Box2i dw = f.dataWindow();
    width = dw.max.x - dw.min.x + 1;
    height = dw.max.y - dw.min.y + 1;
    Array2D<Rgba> pixels;
    pixels.resizeErase(height, width);
    f.setFrameBuffer(&pixels[0][0] - dw.min.x - dw.min.y * width, 1, width);
    f.readPixels(dw.min.y, dw.max.y);
    if (!preallocated) *image = new float[channels*width*height];
    float* im = *image;
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int idx = width*(height-i-1) + j;
            im[3*idx] = pixels[i][j].r;
            if (channels > 1) im[3*idx+1] = pixels[i][j].g;
            if (channels > 2) im[3*idx+2] = pixels[i][j].b;
            if (channels > 3) im[3*idx+3] = pixels[i][j].a;
        }
    }
    return true;
}

bool writeExrImage(const string& filename,
        const float* image,
        int width,
        int height,
        int channels)
{
    Rgba* pixels = new Rgba[width*height];
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int idx = j + (height-i-1)*width;
            pixels[idx].r = image[channels*idx];
            if (channels > 1) {
                pixels[idx].g = image[channels*idx+1];
                pixels[idx].b = image[channels*idx+2];
            } else {
                pixels[idx].g = image[idx];
                pixels[idx].b = image[idx];
            }
            if (channels > 3) {
                pixels[idx].a = image[channels*idx+3];
            } else {
                pixels[idx].a = 1;
            }
        }
    }
    RgbaOutputFile file(filename.c_str(), width, height, WRITE_RGBA);
    file.setFrameBuffer(pixels, 1, width);
    file.writePixels(height);
    delete pixels;
    return true;
}

bool readHdrImage(const string& filename,
        float** image,
        int& width,
        int& height,
        bool preallocated)
{
    rgbe_header_info info;
    FILE* file = fopen(filename.c_str(), "rb");

    RGBE_ReadHeader(file, &width, &height, &info);
    if (!preallocated) *image = new float[3*width*height];
    float* im = *image;
    RGBE_ReadPixels_RLE(file, im, width, height);
    float expadj = info.exposure;
    for (int i = 0; i < width*height*3; ++i) {
        im[i] /= expadj;
    }
    // Flip y-coordinate (standard is 0,0 at top)
    float* row = new float[3*width];
    for (int i = 0; i < height/2; ++i) {
        memcpy(row, im+3*(i*width), 3*width*sizeof(float));
        memcpy(im+3*(i*width), im+3*((height-i-1)*width), 3*width*sizeof(float));
        memcpy(im+3*((height-i-1)*width), row, 3*width*sizeof(float));
    }
    delete [] row;
    fclose(file);
    return true;
}

bool readPngImage(const string& filename,
        char** image,
        int& width,
        int& height,
        bool preallocated)
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
    int bit_depth;
    png_uint_32 pngw, pngh;
    png_get_IHDR(png_ptr, info_ptr, &pngw, &pngh, &bit_depth, &color_type,
                 &interlace_type, NULL, NULL);
    width = pngw;
    height = pngh;

    unsigned int row_bytes = png_get_rowbytes(png_ptr, info_ptr);
    if (!preallocated) *image = new char[width*height*3];
    char* im = *image;

    png_bytepp row_pointers = png_get_rows(png_ptr, info_ptr);

    for (int i = 0; i < height; i++) {
        memcpy(im+(row_bytes * (height-1-i)), row_pointers[i], row_bytes);
    }

    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
    fclose(fp);
    return true;
}
};
