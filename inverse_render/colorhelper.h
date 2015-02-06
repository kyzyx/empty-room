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
        ColorHelper()
            : flip_x(false), flip_y(false)
        {
        }
        ColorHelper(bool image_flip_x, bool image_flip_y)
            : flip_x(image_flip_x), flip_y(image_flip_y)
        {
        }
        ~ColorHelper() {
            for (int i = 0; i < data.size(); ++i) delete [] data[i];
            for (int i = 0; i < cameras.size(); ++i) delete cameras[i];
            for (int i = 0; i < conf.size(); ++i) delete [] conf[i];
            for (int i = 0; i < depth.size(); ++i) delete [] depth[i];
            for (int i = 0; i < labels.size(); ++i) delete [] labels[i];
            for (int i = 0; i < edges.size(); ++i) delete [] edges[i];
        }
        enum {
            READ_COLOR = 1,
            READ_CONFIDENCE = 2,
            READ_DEPTH = 4,
            READ_EDGES = 4,
        };

        bool load(std::string cameraFile, int flags=READ_COLOR);
        bool load(int i, int flags=READ_COLOR);

        void* readImage(int idx);
        void* readDepthMap(int idx);
        void* readConfidenceFile(int idx);
        void* readPngImage(int idx);
        void* readExrImage(int idx);
        void* readHdrImage(int idx);
        bool readCameraFile(const std::string& filename);

        void writeEdgeImages();

        static bool readExrImage(const std::string& filename,
                float*& image,
                int& width,
                int& height,
                int channels=3);
        static bool writeExrImage(const std::string& filename,
                const float* image,
                int width,
                int height,
                int channels=3);
        int size() const { return data.size(); }
        char* getImage(int n) {
            if (n >= data.size()) return NULL;
            return data[n];
        }
        void setLabelImage(int n, char* im) {
            labels[n] = im;
        }
        void setEdges(int n, float* im) {
            edges[n] = im;
        }
        const float* getEdges(int n) {
            if (n >= edges.size()) return NULL;
            return edges[n];
        }
        const char* getLabelImage(int n) {
            if (n >= labels.size()) return NULL;
            return labels[n];
        }
        const float* getConfidenceMap(int n) {
            if (n >= conf.size()) return NULL;
            return conf[n];
        }
        const float* getDepthMap(int n) {
            if (n >= depth.size()) return NULL;
            return depth[n];
        }
        const CameraParams* getCamera(int n) { return cameras[n]; }
    protected:
        void transformAllCameras(const R4Matrix& m);
        void flip(char* a, int w, int h, size_t bytes);

    private:
        bool flip_x, flip_y;
        R4Matrix depth2rgb;
        std::vector<std::string> filenames;
        std::vector<char*> data;   // Raw RGB images, as triples of 32-bit floats
        std::vector<float*> conf;  // Confidence of each image pixel, as floats
        std::vector<float*> depth; // Distance from the camera at each pixel, in mm
        std::vector<char*> labels; // Label of pixel as wall, floor, etc.
        std::vector<float*> edges;
        std::vector<CameraParams*> cameras;
};

#endif
