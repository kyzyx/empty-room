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
    double fov;
};

class ColorHelper {
    public:
        ColorHelper() {
        }
        ~ColorHelper() {
            for (int i = 0; i < data.size(); ++i) delete data[i];
            for (int i = 0; i < cameras.size(); ++i) delete cameras[i];
        }

        bool load(std::string cameraFile, bool read_confidence_file=false);

        bool readImage(const std::string& filename, bool read_confidence_file=false);
        bool readConfidenceFile(const std::string& filename);
        bool readPngImage(const std::string& filename);
        bool readExrImage(const std::string& filename);
        bool readHdrImage(const std::string& filename);
        bool readCameraFile(const std::string& filename);

        static bool writeExrImage(const std::string& filename,
                const float* image,
                int width,
                int height);
        int size() const { return data.size(); }
        const char* getImage(int n) {
            if (n >= data.size()) return NULL;
            return data[n];
        }
        const float* getConfidenceMap(int n) {
            if (n >= conf.size()) return NULL;
            return conf[n];
        }
        const CameraParams* getCamera(int n) { return cameras[n]; }
    protected:
        void transformAllCameras(const R4Matrix& m);

    private:
        std::vector<std::string> filenames;
        std::vector<char*> data;
        std::vector<float*> conf;
        std::vector<CameraParams*> cameras;
};

#endif
