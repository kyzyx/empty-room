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

string replaceExtension(const string& s, string newext) {
    size_t i = s.find_last_of(".");
    if (i == string::npos) return s + newext;
    else return s.substr(0,i+1) + newext;
}
bool endswith(const string& s, string e) {
    if (s.length() > e.length())
        return s.compare(s.length()-e.length(), e.length(), e) == 0;
    else
        return false;
}

bool readDepthMap(const string& filename, void* image, int w, int h) {
    int width = 0;
    int height = 0;
    bool success = readPcdDepthImage(filename, (float**) &image, width, height, true);
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
bool writeScalarMap(const string& filename, const void* image, int w, int h, int bytes) {
    try {
        ofstream out(filename.c_str(), ifstream::binary);
        char* p = (char*) image;
        p += (h-1)*w*bytes;
        for (int i = 0; i < h; ++i) {
            out.write(p,w*bytes);
            p -= w*bytes;
        }
        return true;
    } catch(...) {
        return false;
    }
}

bool readFloatImage(const string& filename, void* image, int& width, int& height, int channels) {
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
        unsigned char* tmp;
        if (!readPngImage(filename, &tmp, width, height)) return false;
        // Upgrade to float 0-1
        float* p = (float*) image;
        for (int i = 0; i < width*height; ++i) {
            for (int j = 0; j < channels; ++j) {
                *(p++) = tmp[i*3+j]/255.;
            }
        }
        delete tmp;
    } else if (endswith(filename, ".bin")) {
        unsigned char* tmp;
        if (!readBinaryImage(filename, &tmp, width, height)) return false;
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

    return true;
}

bool readRGBImage(const string& filename, void* image, int& width, int& height, int channels) {
    if (channels == 3) {
        if (endswith(filename, ".png")) {
            if (!readPngImage(filename, (unsigned char**) &image, width, height, true))
                return false;
        }
        else if (endswith(filename, ".bin")) {
            if (!readBinaryImage(filename, (unsigned char**) &image, width, height, true))
                return false;
        }
    } else {
        unsigned char* tmp;
        if (endswith(filename, ".png")) {
            if (!readPngImage(filename, &tmp, width, height))
                return false;
        }
        else if (endswith(filename, ".bin")) {
            if (!readBinaryImage(filename, &tmp, width, height))
                return false;
        } else {
            return false;
        }
        unsigned char* p = (unsigned char*) image;
        for (int i = 0; i < width*height; ++i) {
            for (int j = 0; j < channels; ++j) {
                *(p++) = tmp[i*3+j];
            }
        }
        delete tmp;
    }

    return true;
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
            int pidx = j + i*width;
            int iidx = j + (height-i-1)*width;
            pixels[pidx].r = image[channels*iidx];
            if (channels > 1) {
                pixels[pidx].g = image[channels*iidx+1];
                pixels[pidx].b = image[channels*iidx+2];
            } else {
                pixels[pidx].g = image[iidx];
                pixels[pidx].b = image[iidx];
            }
            if (channels > 3) {
                pixels[pidx].a = image[channels*iidx+3];
            } else {
                pixels[pidx].a = 1;
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
        unsigned char** image,
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
    if (!preallocated) *image = new unsigned char[width*height*3];
    char* im = (char*) *image;

    png_bytepp row_pointers = png_get_rows(png_ptr, info_ptr);

    for (int i = 0; i < height; i++) {
        memcpy(im+(row_bytes * (height-1-i)), row_pointers[i], row_bytes);
    }

    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
    fclose(fp);
    return true;
}
bool writePngImage(const string& filename,
        const unsigned char* image,
        int width,
        int height,
        int channels)
{
    png_structp png_ptr;
    png_infop info_ptr;
    unsigned int sig_read = 0;
    int color_type, interlace_type;
    png_bytep * row_pointers;

    FILE *fp;
    if ((fp = fopen(filename.c_str(), "wb")) == NULL)
        return false;

    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr) {
        fclose(fp);
        return false;
    }

    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        fclose(fp);
        return false;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        fclose(fp);
        return false;
    }

    png_init_io(png_ptr, fp);

    if (setjmp(png_jmpbuf(png_ptr))) {
        fclose(fp);
        return false;
    }

    png_set_IHDR(png_ptr, info_ptr, width, height,
            8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
            PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

    png_write_info(png_ptr, info_ptr);

    if (setjmp(png_jmpbuf(png_ptr))) {
        fclose(fp);
        return false;
    }

    row_pointers = (png_bytep*) malloc(sizeof(png_bytep) * height);
    unsigned int row_bytes = png_get_rowbytes(png_ptr,info_ptr);
    const unsigned char* im = image;
    for (int i = 0; i < height; i++) {
        row_pointers[i] = (png_byte*) malloc(row_bytes);
        if (channels == 3) {
            memcpy(row_pointers[i], im + (row_bytes * (height-1-i)), row_bytes);
        } else {
            for (int j = 0; j < width; ++j) {
                int idx = j + width*(height-1-i);
                for (int k = 0; k < 3; ++k)
                    row_pointers[i][3*j+k] = k<channels?im[channels*idx+k]:im[channels*idx+channels-1];
            }
        }
    }

    png_write_image(png_ptr, row_pointers);

    if (setjmp(png_jmpbuf(png_ptr))) {
        fclose(fp);
        return false;
    }

    png_write_end(png_ptr, NULL);

    for (int y=0; y<height; y++)
        free(row_pointers[y]);
    free(row_pointers);

    fclose(fp);
    return true;
}

bool readBinaryImage(const std::string& filename,
        unsigned char** image,
        int& width,
        int& height,
        bool preallocated)
{
        std::map<std::string, float> header;
        return readBinaryImageWithHeader(filename, image, width, height, header, preallocated);
}

bool readBinaryImageWithHeader(const std::string& filename,
        unsigned char** image,
        int& width,
        int& height,
        std::map<std::string, float>& header,
        bool preallocated)
{
    try {
        ifstream in(filename, ios::in | ios::binary);
        string line;
        getline(in, line);
        bool bgr = false;
        int bpp = 0;
        while (line != "end_header") {
            int n = line.find(' ');
            if (n != string::npos) {
                string key = line.substr(0, n);
                if (key == "width") width = atoi(line.substr(n+1).c_str());
                else if (key == "height") height = atoi(line.substr(n+1).c_str());
                else if (key == "format") bgr = line.substr(n+1) == "BGR";
                else if (key == "cameraModelName") ;
                else header[key] = atof(line.substr(n+1).c_str());
            }
            getline(in, line);
        }
        if (!preallocated) *image = new unsigned char[width*height*3];
        char* im = (char*) *image;
        for (int i = 0; i < height; i++) {
            in.read(im, width*3);
            im += width*3;
        }
        if (bgr) {
            for (int i = 0; i < width*height*3; i += 3) {
                swap((*image)[i], (*image)[i+2]);
            }
        }
        return true;
    } catch(...) {
        return false;
    }
}
bool readImageHeader(const std::string& filename,
        int& width,
        int& height,
        std::map<std::string, float>& header)
{
    try {
        ifstream in(filename, ios::in | ios::binary);
        string line;
        getline(in, line);
        bool bgr = false;
        int bpp = 0;
        while (line != "end_header") {
            int n = line.find(' ');
            if (n != string::npos) {
                string key = line.substr(0, n);
                if (key == "width") width = atoi(line.substr(n+1).c_str());
                else if (key == "height") height = atoi(line.substr(n+1).c_str());
                else if (key == "format") bgr = line.substr(n+1) == "BGR";
                else if (key == "cameraModelName") ;
                else header[key] = atof(line.substr(n+1).c_str());
            }
            getline(in, line);
        }
        return true;
    } catch(...) {
        return false;
    }
}
};
