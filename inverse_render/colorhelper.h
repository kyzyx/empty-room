#ifndef _COLOR_HELPER_H
#define _COLOR_HELPER_H

#include "R3Shapes/R3Shapes.h"
#include <string>
#include <vector>


struct CameraParams {
    R3Point pos;
    R3Vector up;
    R3Vector towards;
    R3Vector right;

    int width;
    int height;
    double focal_length;
};

class ColorHelper {
    public:
        ColorHelper() {
        }
        ~ColorHelper() {
            for (int i = 0; i < data.size(); ++i) delete data[i];
            for (int i = 0; i < cameras.size(); ++i) delete cameras[i];
        }

        bool load(std::string imageListFile, std::string cameraFile);

        bool readImageNames(const std::string& filename);
        bool readImage(const std::string& filename);
        bool readPngImage(const std::string& filename);
        bool readHdrImage(const std::string& filename);
        bool readMayaCameraFile(const std::string& filename);

        int size() const { return data.size(); }
        const char* getImage(int n) { return data[n]; }
        const CameraParams* getCamera(int n) { return cameras[n]; }
    protected:

    private:
        std::vector<std::string> filenames;
        std::vector<char*> data;
        std::vector<CameraParams*> cameras;
};

#endif
