#ifndef _IMAGE_IO_H
#define _IMAGE_IO_H
#endif

#include <string>
#include <map>

namespace ImageIO {
std::string replaceExtension(const std::string& s, std::string newext);
bool readDepthMap(const std::string& filename, void* image, int w, int h);
bool readScalarMap(const std::string& filename, void* image, int w, int h, int bytes);
bool writeScalarMap(const std::string& filename, const void* image, int w, int h, int bytes);
bool readFloatImage(const std::string& filename, void* image, int w, int h, int channels=3);
bool readRGBImage(const std::string& filename, void* image, int w, int h, int channels=3);

// Particular file formats
bool readPcdDepthImage(const std::string& filename,
        float** image,
        int& width,
        int& height,
        bool preallocated=false);
bool readExrImage(const std::string& filename,
        float** image,
        int& width,
        int& height,
        int channels=3,
        bool preallocated=false);
bool writeExrImage(const std::string& filename,
        const float* image,
        int width,
        int height,
        int channels=3);
bool readHdrImage(const std::string& filename,
        float** image,
        int& width,
        int& height,
        bool preallocated=false);
bool readPngImage(const std::string& filename,
        unsigned char** image,
        int& width,
        int& height,
        bool preallocated=false);
bool writePngImage(const std::string& filename,
        const unsigned char* image,
        int width,
        int height,
        int channels=3);
bool readBinaryImage(const std::string& filename,
        unsigned char** image,
        int& width,
        int& height,
        bool preallocated=false);
bool readBinaryImageWithHeader(const std::string& filename,
        unsigned char** image,
        int& width,
        int& height,
        std::map<std::string, float>& header,
        bool preallocated=false);
};
